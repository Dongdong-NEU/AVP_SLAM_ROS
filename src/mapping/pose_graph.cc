/**
 * @file pose_graph.cc
 * @brief 全局位姿图 - SLAM后端核心
 *
 * 核心职责：
 *   1. 维护所有轨迹节点和子图的全局位姿
 *   2. 构建子图-节点之间的约束（局部约束 + 全局回环约束）
 *   3. 检测回环闭合（通过距离阈值 + 扫描匹配验证）
 *   4. 调用 OptimizationProblem（Ceres SPA）进行全局优化
 *
 * 位姿图结构示意：
 *
 *   Node_0 ──(局部约束)──► Submap_0
 *   Node_1 ──(局部约束)──► Submap_0, Submap_1
 *   Node_2 ──(局部约束)──► Submap_1
 *     ...
 *   Node_N ──(回环约束)──► Submap_0     ← 回环检测发现距离近，扫描匹配验证通过
 *
 *   SPA优化：最小化所有约束的残差，调整所有节点和子图的全局位姿
 */

#include "pose_graph.h"

namespace AVP
{
namespace mapping
{

/**
 * 添加新的轨迹节点到位姿图中
 *
 * 流程：
 *   1. 计算局部坐标→全局坐标的变换，将节点位姿转到全局坐标系
 *   2. 如果有新的子图，记录到 submap_data 中
 *   3. 检查前一个子图是否已完成（insertion_finished）
 *   4. 为新节点计算约束（局部约束 + 回环检测）
 */
void PoseGraph::AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data, 
                                        const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
{
    // 获取 局部坐标系→全局坐标系 的变换矩阵
    // 注意：当前实现中 global_submap_poses 从未被更新（RunOptimization 更新的是
    // global_submap_poses_2d），因此此函数始终返回 Identity 矩阵。
    // 这意味着新添加节点的 global_pose 等于 local_pose，
    // 真正的全局位姿校正在 RunOptimization() 中统一完成。
    Eigen::Matrix4d T_global_local = ComputeLocalToGlobalTransform();

    // 将节点的局部位姿变换到全局坐标系
    Eigen::Vector4d P_local_node;
    P_local_node << constant_data->local_pose, 1;
    auto iter_node = data_.trajectory_nodes.insert(std::make_pair(data_.trajectory_nodes.size(), 
                                                 TrajectoryNode{constant_data, (T_global_local*P_local_node).head(3)}));
    
    // 如果 insertion_submaps 的最后一个子图是新的（之前没见过），则注册到位姿图中
    if (data_.submap_data.size() == 0 || data_.submap_data.rbegin()->second.submap != insertion_submaps.back())
    {
        InternalSubmapData temp;
        auto ret = data_.submap_data.insert(std::make_pair(data_.submap_data.size(), temp));
        if (ret.second == true)
        {
            ret.first->second.submap = insertion_submaps.back();
        }
    }

    // 检查第一个（较旧的）子图是否已经完成数据插入
    // ActiveSubmaps 维护两个子图：front=旧的（可能已满），back=新的
    const bool newly_finished_submap = insertion_submaps.front()->insertion_finished();

    const unsigned int node_id = iter_node.first->first;
    SlamFileLogger::Instance().Logf(
        "NODE",
        "added node_id=%u local_pose=(%.3f, %.3f, %.3f) submaps=%zu newly_finished_submap=%d",
        node_id,
        constant_data->local_pose[0], constant_data->local_pose[1], constant_data->local_pose[2],
        insertion_submaps.size(), newly_finished_submap ? 1 : 0);

    ComputeConstraintsForNode(node_id, insertion_submaps, newly_finished_submap);
}

/**
 * 回环检测：检查当前节点是否与某个已完成的子图形成回环
 *
 * 检测策略（简化版）：
 *   1. 只检查已完成的子图（insertion_finished == true）
 *   2. 避免重复检测（submap_node_has_looped 记录已检测的对）
 *   3. 计算节点与子图之间的距离，超过 5m 则跳过
 *   4. 距离阈值内的标记为候选回环对，等待 RunOptimization 时做扫描匹配验证
 *
 * 注意：实际的扫描匹配验证在 RunOptimization() 中执行，
 *       这里只做距离预筛选，避免每帧都做耗时的扫描匹配
 */
void PoseGraph::DetectLoopAndComputeConstraint(unsigned int submap_id, unsigned int node_id)
{
    const TrajectoryNode::Data* constant_data;
    const Submap* submap;
    {
        // 只对已完成插入的子图做回环检测
        if (!data_.submap_data.at(submap_id).submap->insertion_finished())
        {
            SlamFileLogger::Instance().Logf("LOOP", "skip submap_id=%u: insertion not finished", submap_id);
            return;
        }

        // 每个 submap 只做一次回环检测，避免重复
        if (submap_node_has_looped.count(submap_id))
        {
            return;
        }
        
        // 计算节点相对于子图的相对位姿 T_submap_node
        Eigen::Vector3d initial_relative_pose;
        initial_relative_pose = ComputeRelativePose(optimization_problem_->submap_data().at(submap_id).global_pose,
                                                    optimization_problem_->node_data().at(node_id).global_pose_2d);

        // 距离阈值筛选：超过 5m 则认为不可能是回环
        double distance = initial_relative_pose.head<2>().norm();
        if ( distance > 5)
        {
            return;
        }

        SlamFileLogger::Instance().Logf(
            "LOOP",
            "candidate loop submap_id=%u node_id=%u distance=%.3f",
            submap_id, node_id, distance);
        submap_node_has_looped.insert(std::make_pair(submap_id, node_id));
    }
}

/**
 * 为新添加的节点计算所有约束
 *
 * 两类约束：
 *   1. 局部约束 (Non_Global)：节点与其所属子图之间的相对位姿约束
 *      → 来源于扫描匹配的结果，权重较小 (translation=5e2, rotation=1.6e3)
 *
 *   2. 全局约束 (Global)：回环闭合约束
 *      → 来源于 DetectLoopAndComputeConstraint，权重较大 (translation=1.1e4, rotation=1e5)
 *      → 权重大意味着优化器更信任回环约束
 */
void PoseGraph::ComputeConstraintsForNode(const unsigned int node_id, std::vector<std::shared_ptr<const Submap>> insertion_submaps, const bool newly_finished_submap)
{
    // 正在运行全局优化时，跳过约束计算（避免数据竞争）
    if (running_optimization == true)
    {
        return;
    }
    
    std::vector<unsigned int> submap_ids;
    std::vector<unsigned int> finished_submap_ids;
    std::set<unsigned int> newly_finished_submap_node_ids;
    {
        // 初始化/获取子图在优化问题中的ID
        submap_ids = InitializeGlobalSubmapPoses(insertion_submaps);

        // 将节点添加到优化问题中
        Eigen::Vector3d node_local_pose = data_.trajectory_nodes.find(node_id)->second.constant_data->local_pose;
        optimization_problem_->AddTrajectoryNode(node_local_pose, node_local_pose);
    
        // 为每个关联的子图建立局部约束
        // 相对位姿 = T_submap^{-1} * T_node，即节点在子图坐标系下的位姿
        for (size_t i = 0; i < submap_ids.size(); i++)
        {
            data_.submap_data.find(submap_ids[i])->second.node_ids.emplace(node_id);
            Eigen::Vector3d relative_transform = ComputeRelativePose(insertion_submaps[i]->local_pose(), data_.trajectory_nodes.find(node_id)->second.constant_data->local_pose);
            data_.constraints.push_back(Constraint{submap_ids[i], node_id, 5e2, 1.6e3, relative_transform, Constraint::Tag::Non_Global});
        }

        // 收集所有已完成的子图ID，用于后续回环检测
        for(auto& elem : data_.submap_data)
        {
            if (elem.second.state == SubmapState::kFinished)
            {
                finished_submap_ids.push_back(elem.first);
            }
        }

        // 如果有新完成的子图，标记其状态
        if (newly_finished_submap)
        {
            data_.submap_data.find(submap_ids.front())->second.state = SubmapState::kFinished;
            newly_finished_submap_node_ids = data_.submap_data.find(submap_ids.front())->second.node_ids;
        }
    }

    // 对所有已完成的子图，尝试与当前节点做回环检测
    for(auto& elem : finished_submap_ids)
    {
        DetectLoopAndComputeConstraint(elem, node_id);
    }
}

/**
 * 运行全局优化（由 GlobalTrajectoryBuilder 每5秒定时触发）
 *
 * 流程：
 *   1. 对所有候选回环对，用 FastCorrelativeScanMatcher（分支定界）做扫描匹配验证
 *      → 匹配得分 > 0.7 才认为是有效回环，添加全局约束
 *   2. 调用 OptimizationProblem::Solve 执行 Ceres SPA 优化
 *      → 最小化所有约束（局部 + 全局）的残差
 *   3. 将优化后的位姿更新到所有轨迹节点
 *   4. 发布优化后的轨迹到 /path_estimated_after_spa
 */
void PoseGraph::RunOptimization()
{
    running_optimization = true;

    const size_t loop_candidates = submap_node_has_looped.size();
    size_t loop_accepted = 0;
    SlamFileLogger::Instance().Logf(
        "OPTIMIZE", "RunOptimization start: loop_candidates=%zu constraints=%zu",
        loop_candidates, data_.constraints.size());

    const TrajectoryNode::Data* constant_data;
    const Submap* submap;
    for(auto elem : submap_node_has_looped)
    {
        constant_data = data_.trajectory_nodes.at(elem.second).constant_data.get();
        submap = static_cast<const Submap*>(data_.submap_data.at(elem.first).submap.get());

        // 计算基于里程计的初始相对位姿（用于几何一致性检查）
        const Eigen::Vector3d initial_relative_pose = ComputeRelativePose(
            optimization_problem_->submap_data().at(elem.first).global_pose,
            optimization_problem_->node_data().at(elem.second).global_pose_2d);

        FastCorrelativeScanMatcher2D fast_csm(*(submap->grid()));
        float score = 0;
        Eigen::Vector3d pose_estimated = Eigen::Vector3d::Zero();

        constexpr float kMinLoopScore = 0.75f;
        constexpr double kMaxTranslationDeviation = 8;

        if (fast_csm.Match(constant_data->local_pose,
                           constant_data->filtered_semantic_data,
                           kMinLoopScore, &score, &pose_estimated))
        {
            Eigen::Vector3d constraint_transform = ComputeRelativePose(submap->local_pose(), pose_estimated);

            // 几何一致性验证：匹配得到的相对位姿 vs 里程计估计的相对位姿
            // 差距过大说明扫描匹配到了错误位置（如停车场中相邻平行车道）
            const double translation_deviation =
                (constraint_transform.head<2>() - initial_relative_pose.head<2>()).norm();

            if (translation_deviation > kMaxTranslationDeviation)
            {
                SlamFileLogger::Instance().Logf(
                    "LOOP",
                    "rejected loop submap_id=%u node_id=%u score=%.3f deviation=%.3f>%.1f "
                    "matched_rel=(%.3f,%.3f,%.3f) odom_rel=(%.3f,%.3f,%.3f)",
                    elem.first, elem.second, score, translation_deviation, kMaxTranslationDeviation,
                    constraint_transform[0], constraint_transform[1], constraint_transform[2],
                    initial_relative_pose[0], initial_relative_pose[1], initial_relative_pose[2]);
                continue;
            }

            data_.constraints.push_back(Constraint{elem.first, elem.second, 1.1e4, 1e5,
                                                constraint_transform, Constraint::Tag::Global});
            ++loop_accepted;
            SlamFileLogger::Instance().Logf(
                "LOOP",
                "accepted loop submap_id=%u node_id=%u score=%.3f deviation=%.3f "
                "relative_pose=(%.3f, %.3f, %.3f)",
                elem.first, elem.second, score, translation_deviation,
                constraint_transform[0], constraint_transform[1], constraint_transform[2]);
        }
        else
        {
            SlamFileLogger::Instance().Logf(
                "LOOP",
                "rejected loop submap_id=%u node_id=%u score=%.3f (threshold=%.2f)",
                elem.first, elem.second, score, kMinLoopScore);
        }
    }

    if (optimization_problem_->submap_data().empty())
    {
        running_optimization = false;
        SlamFileLogger::Instance().Log("OPTIMIZE", "skip SPA: no submaps registered");
        return;
    }

    const size_t constraints_before_spa = data_.constraints.size();
    optimization_problem_->Solve(data_.constraints);

    num_nodes_since_last_loop_closure_ = 0;

    const auto& submap_data = optimization_problem_->submap_data();
    const auto& node_data = optimization_problem_->node_data();

    // 阶段3: 将优化后的位姿更新到轨迹节点，并构造优化后的路径消息
    nav_msgs::Path path_estimated_after_spa;

    for(const auto& elem : node_data)
    {
        data_.trajectory_nodes.at(elem.first).global_pose = elem.second.global_pose_2d;
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = elem.second.global_pose_2d[0];
        temp.pose.position.y = elem.second.global_pose_2d[1];
        temp.pose.orientation.z = elem.second.global_pose_2d[2];
        path_estimated_after_spa.poses.push_back(temp);
    }

    // 发布全局优化后的轨迹
    path_estimated_after_spa.header.frame_id = "world";
    path_estimated_after_spa_pub.publish(path_estimated_after_spa);
    data_.global_submap_poses_2d = submap_data;

    for(const auto& node : node_data)
    {
        data_.trajectory_nodes.at(node.first).global_pose = node.second.global_pose_2d;
    }

    running_optimization = false;

    if (!node_data.empty())
    {
        const auto& first_node = node_data.begin()->second.global_pose_2d;
        const auto& last_node = node_data.rbegin()->second.global_pose_2d;
        SlamFileLogger::Instance().Logf(
            "OPTIMIZE",
            "SPA done: loop_accepted=%zu/%zu constraints=%zu nodes=%zu first_pose=(%.3f,%.3f,%.3f) last_pose=(%.3f,%.3f,%.3f)",
            loop_accepted, loop_candidates, constraints_before_spa, node_data.size(),
            first_node[0], first_node[1], first_node[2],
            last_node[0], last_node[1], last_node[2]);
    }
}


/**
 * 初始化子图在优化问题中的全局位姿
 *
 * ActiveSubmaps 维护最多2个子图 [front=旧, back=新]
 * 此函数根据 insertion_submaps 的状态，决定哪些子图需要注册到优化问题中
 *
 * 三种情况：
 *   - size==1 且 首次：注册第一个子图
 *   - 新子图出现（back指针变了）：注册新子图，返回 [旧ID, 新ID]
 *   - 子图未变化：返回 [前一个ID, 当前ID]
 */
std::vector<unsigned int> PoseGraph::InitializeGlobalSubmapPoses(const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
{
    const auto& submap_data = optimization_problem_->submap_data();

    // 只有一个子图（系统刚启动时）
    if (insertion_submaps.size() == 1)
    {
        if (submap_data.size()==0)
        {
            optimization_problem_->AddSubmap(insertion_submaps.front()->local_pose());
        }
        return {0};
    }
    
    // 检查是否需要注册新的子图
    unsigned int last_submap_id = std::prev(submap_data.end())->first;
    if (data_.submap_data.find(last_submap_id)->second.submap == insertion_submaps.front())
    {
        // 新的子图（back）还没注册过
        optimization_problem_->AddSubmap(insertion_submaps.back()->local_pose());
        return {last_submap_id, last_submap_id+1};
    }

    // 两个子图都已注册
    return {last_submap_id-1, last_submap_id};   
}


/**
 * 计算 局部坐标系→全局坐标系 的变换矩阵
 *
 * 公式: T_global_local = T_global_submap * T_local_submap^{-1}
 *
 * 注意：此函数查询的是 data_.global_submap_poses（Eigen::Vector3d map），
 * 但 RunOptimization() 更新的是 data_.global_submap_poses_2d（SubmapSpec2D map）。
 * 由于 global_submap_poses 从未被写入，此函数当前始终返回 Identity。
 * 如需启用全局坐标转换，应改为从 global_submap_poses_2d 读取。
 */
Eigen::Matrix4d PoseGraph::ComputeLocalToGlobalTransform()
{
    std::unique_lock<std::mutex> lock(mMutex_data_);

    // global_submap_poses 始终为空（见上方说明），返回单位矩阵
    if (data_.global_submap_poses.empty())
    {
        return Eigen::Matrix4d::Identity();
    }

    // 使用最后一个子图的全局位姿和局部位姿计算变换
    auto iter = data_.global_submap_poses.rbegin();
    
    Eigen::Matrix4d T_global_submap;
    T_global_submap << cos(iter->second[2]), -1*sin(iter->second[2]), 0, iter->second[0],
                       sin(iter->second[2]), cos(iter->second[2]), 0, iter->second[1],
                       0,0,1,0,
                       0,0,0,1;

    Eigen::Vector3d submap_local_pose = data_.submap_data.find(iter->first)->second.submap->local_pose();

    Eigen::Matrix4d T_local_submap;
    T_local_submap << cos(submap_local_pose[2]), -1*sin(submap_local_pose[2]), 0, submap_local_pose[0],
                       sin(submap_local_pose[2]), cos(submap_local_pose[2]), 0, submap_local_pose[1],
                       0,0,1,0,
                       0,0,0,1;  

    Eigen::Matrix4d ret = T_global_submap * T_local_submap.inverse();
    return ret;
}

// 返回所有轨迹节点（带锁保护，供外部读取）
std::map<unsigned int, TrajectoryNode> PoseGraph::getSubmapList()
{
    std::unique_lock<std::mutex> lock(mMutex_data_);
    return data_.trajectory_nodes;
}

size_t PoseGraph::GetConstraintCount() const
{
    return data_.constraints.size();
}

size_t PoseGraph::GetGlobalConstraintCount() const
{
    size_t count = 0;
    for (const auto& constraint : data_.constraints)
    {
        if (constraint.tag == Constraint::Tag::Global)
        {
            ++count;
        }
    }
    return count;
}

size_t PoseGraph::GetNodeCount() const
{
    return data_.trajectory_nodes.size();
}

size_t PoseGraph::GetSubmapCount() const
{
    return data_.submap_data.size();
}

} // namespace mapping
} // namespace AVP
