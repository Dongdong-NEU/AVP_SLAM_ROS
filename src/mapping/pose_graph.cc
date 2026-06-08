#include "pose_graph.h"

namespace AVP
{
namespace mapping
{

/**
 * @brief 在位姿图中新增一个轨迹节点，并维护与子地图的对应关系和约束。
 * @param constant_data 轨迹节点的常量数据，包含局部位姿和点云数据
 * @param insertion_submaps 与该节点相关联的子地图列表
 * 
 * 这是SLAM后端的核心函数，负责：
 * 1. 将局部轨迹节点转换为全局坐标系
 * 2. 管理子地图数据结构
 * 3. 计算节点间的约束关系
 * 4. 为后端优化提供数据基础
 */
void PoseGraph::AddNode(std::shared_ptr<const TrajectoryNode::Data> constant_data, 
                                        const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
{
    // === 步骤1: 计算局部到全局的坐标变换矩阵 ===
    Eigen::Matrix4d T_global_local = ComputeLocalToGlobalTransform();
    
    // === 步骤2: 将局部位姿转换为全局位姿 ===
    Eigen::Vector4d P_local_node;
    P_local_node << constant_data->local_pose, 1;  // 齐次坐标形式 [x, y, theta, 1]
    
    // 插入新的轨迹节点（轨迹节点中有降采样过后的点云与local_pose）到位姿图中
    // 节点ID = 当前节点数量，全局位姿 = T_global_local * 局部位姿
    // iter_node 的形式： 
    //     std::pair< std::map<int, TrajectoryNode>::iterator, // 迭代器，指向插入元素
    //     bool                                     // 是否插入成功
    // >
    auto iter_node = data_.trajectory_nodes.insert(std::make_pair(data_.trajectory_nodes.size(), 
                                                 TrajectoryNode{constant_data, (T_global_local*P_local_node).head(3)}));
    
    // === 步骤3: 管理子地图数据结构 ===
    // 如果当前data_.submap_data.size() == 0 或者最新子地图不是当前插入的子地图，则创建新的内部子地图数据结构
    if (data_.submap_data.size() == 0 || data_.submap_data.rbegin()->second.submap != insertion_submaps.back())
    {
        // 创建新的内部子地图数据结构
        InternalSubmapData temp;
        auto ret = data_.submap_data.insert(std::make_pair(data_.submap_data.size(), temp));
        
        // 如果插入成功，关联子地图指针
        if (ret.second == true)
        {
            ret.first->second.submap = insertion_submaps.back();
        }
    }

    // === 步骤4: 检查子地图完成状态 ===
    // 判断前端子地图是否已经完成构建（不再接受新的点云数据）、
    // insertion_submaps 这是从前端传入的子地图列表，通常包含1-2个子地图，在这个SLAM系统中，新的轨迹节点通常会同时插入到多个子地图中（当前活跃的子地图）
    // 获取列表中的第一个子地图
    // 在SLAM中，这通常是较旧的子地图，可能即将完成构建，返回true表示子地图已经完成构建，不再接受新的点云数据，false表示子地图仍在构建中可以继续插入新的点云数据
    // submap：活跃状态 → 完成状态 → 用于回环检测（只有完成构建的子地图才能用于回环检测？确保地图数据的稳定性和一致性 避免在构建过程中进行不准确的匹配）
    const bool newly_finished_submap = insertion_submaps.front()->insertion_finished();

    // === 步骤5: 计算约束关系 ===
    // 为新节点计算与子地图之间的约束关系，这些约束将用于后端优化
    // iter_node.first->first：trajtoryNode 索引， 子图， 
    ComputeConstraintsForNode(iter_node.first->first, insertion_submaps, newly_finished_submap);
    
    // 调试信息：输出当前子地图和节点的数量
    // ROS_INFO("submap size: %d, node size: %d", data_.submap_data.size(), data_.trajectory_nodes.size());

}

/**
 * @brief 检测回环并计算全局约束
 * @param submap_id 子地图ID
 * @param node_id 轨迹节点ID
 * 
 * 这是SLAM后端回环检测的核心函数，负责：
 * 1. 检查子地图和节点是否满足回环条件
 * 2. 基于距离进行初步筛选
 * 3. 记录潜在的回环候选对
 * 4. 为后续的精确回环验证做准备
 */
void PoseGraph::DetectLoopAndComputeConstraint(unsigned int submap_id, unsigned int node_id)
{
    const TrajectoryNode::Data* constant_data;
    const Submap* submap;
    {
        // === 步骤1: 检查子地图完成状态 ===
        // 只有完成构建的子地图才能用于回环检测，确保地图数据稳定
        if (!data_.submap_data.at(submap_id).submap->insertion_finished())
        {
            ROS_INFO("submap %d has not finished", submap_id);
            return;
        }

        // === 步骤2: 避免重复检测 ===
        // 如果这个子地图已经检测过回环，跳过以提高效率
        if (submap_node_has_looped.count(submap_id))
        {
            return;
        }
        
        // === 步骤3: 计算初步的相对位姿 ===
        Eigen::Vector3d initial_relative_pose; // T_submap_node; 子地图到节点的相对变换
        // 基于当前的全局位姿估计计算子地图与节点的相对位姿
        initial_relative_pose = ComputeRelativePose(optimization_problem_->submap_data().at(submap_id).global_pose,
                                                    optimization_problem_->node_data().at(node_id).global_pose_2d);

        // === 步骤4: 距离筛选 ===
        // 计算子地图与节点之间的欧几里得距离（仅考虑x, y坐标）
        double distance = initial_relative_pose.head<2>().norm();
        
        // 距离阈值筛选：如果距离超过5米，认为不太可能是回环
        if ( distance > 5)
        {
            return;
        }

        // === 步骤5: 记录回环候选 ===
        // 通过初步筛选的子地图-节点对，记录为潜在回环候选
        ROS_INFO("submap id: %d, node id: %d distance: %f: ", submap_id, node_id, distance);
        submap_node_has_looped.insert(std::make_pair(submap_id, node_id));
    }
    
    // === 注释掉的代码：精确回环验证和约束生成 ===
    // 以下代码实现了基于扫描匹配的精确回环验证，目前被注释掉
    // 
    // 完整的回环检测流程应该包括：
    // 1. 获取节点的点云数据和子地图的栅格地图
    // 2. 使用实时相关扫描匹配器进行精确位姿估计
    // 3. 基于匹配得分判断是否为真正的回环
    // 4. 如果确认回环，生成全局约束并添加到约束图中
    
    //     constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    //     submap = static_cast<const Submap*>(data_.submap_data.at(submap_id).submap.get());
    //     Eigen::Vector3d pose_estimated = Eigen::Vector3d::Zero();
    //     Eigen::Vector3d initial_pose = constant_data->local_pose;
 
    // // 可选：使用快速相关扫描匹配器进行初步匹配
    // // FastCorrelativeScanMatcher2D* fast_correlative_scan_matcher = new FastCorrelativeScanMatcher2D(*submap->grid());
    // RealTimeCorrelativeScanMatcher real_csm;
    // float score = 0;
  
    // // 执行扫描匹配，验证回环假设
    // if(real_csm.Match(constant_data->local_pose,
    //                   constant_data->filtered_semantic_data,
    //                   *(submap->grid()),pose_estimated))
    // {
    //     // 确认回环，设置优化标志
    //     running_optimization = true;
 
    //     // 调试信息：输出回环检测结果
    //     // ROS_INFO("detect loop closure, submap_id: %d, node_id: %d, score: %f,\n predict node pose: %f %f %f  after fast-CSM: %f %f %f", submap_id, node_id, score,
    //     // initial_pose[0], initial_pose[1], initial_pose[2], pose_estimated[0], pose_estimated[1], pose_estimated[2]);
    //     
    //     // 计算约束变换并添加全局约束
    //     Eigen::Vector3d constraint_transform = ComputeRelativePose(submap->local_pose(), pose_estimated);
    //     data_.constraints.push_back(Constraint{submap_id, node_id, 1.1e4, 1e5,
    //                                            constraint_transform, Constraint::Tag::Global});
    //     // 可选：立即触发优化
    //     // RunOptimization();
    // }
    // // ROS_INFO("loop detect fail");
}

/**
 * @brief 为新添加的轨迹节点计算约束关系
 * @param node_id 新添加的轨迹节点ID
 * @param insertion_submaps 与该节点相关联的子地图列表
 * @param newly_finished_submap 是否有子地图刚刚完成构建
 * 
 * 这个函数是SLAM后端约束构建的核心，负责：
 * 1. 建立节点与子地图之间的局部约束（里程计约束）
 * 2. 检测回环并建立全局约束
 * 3. 管理子地图状态转换
 * 4. 为图优化准备约束数据
 */
void PoseGraph::ComputeConstraintsForNode(const unsigned int node_id, std::vector<std::shared_ptr<const Submap>> insertion_submaps, const bool newly_finished_submap)
{
    // === 步骤1: 检查优化状态 ===
    // 如果正在进行优化，跳过约束计算，避免数据竞争
    if (running_optimization == true)
    {
        return;
    }
    
    // === 步骤2: 初始化数据结构 ===
    std::vector<unsigned int> submap_ids;                    // 当前节点关联的子地图ID列表
    std::vector<unsigned int> finished_submap_ids;           // 处于完成状态的子图id的集合
    std::set<unsigned int> newly_finished_submap_node_ids;   // 新完成子地图包含的节点ID集合
    
    {
        // === 步骤3: 初始化全局子地图位姿 ===
        // 获取与当前插入子地图对应的子地图ID列表
        submap_ids = InitializeGlobalSubmapPoses(insertion_submaps);

        // === 步骤4: 将节点添加到优化问题中 ===
        Eigen::Vector3d node_local_pose = data_.trajectory_nodes.find(node_id)->second.constant_data->local_pose;
        optimization_problem_->AddTrajectoryNode(node_local_pose, node_local_pose);
    
        // === 步骤5: 建立局部约束（里程计约束）===
        // 为每个相关联的子地图建立与当前节点的约束关系
        for (size_t i = 0; i < submap_ids.size(); i++)
        {
            // 记录哪些节点属于哪个子地图
            data_.submap_data.find(submap_ids[i])->second.node_ids.emplace(node_id);
            
            // 计算子地图到节点的相对变换
            Eigen::Vector3d relative_transform = ComputeRelativePose(insertion_submaps[i]->local_pose(), 
                                                                   data_.trajectory_nodes.find(node_id)->second.constant_data->local_pose);
            
            // 添加局部约束：子地图ID，节点ID，平移权重，旋转权重，相对变换，约束类型
            data_.constraints.push_back(Constraint{submap_ids[i], node_id, 5e2, 1.6e3, relative_transform, Constraint::Tag::Non_Global});
        }

        // === 步骤6: 收集已完成的子地图 ===
        // 遍历所有子地图，找出处于完成状态的子地图
        for(auto& elem : data_.submap_data)
        {
            if (elem.second.state == SubmapState::kFinished)
            {
                finished_submap_ids.push_back(elem.first);
            }
        }

        // === 步骤7: 处理新完成的子地图 ===
        // 如果有子地图刚刚完成，更新其状态并记录相关节点
        if (newly_finished_submap)
        {
            data_.submap_data.find(submap_ids.front())->second.state = SubmapState::kFinished;
            newly_finished_submap_node_ids = data_.submap_data.find(submap_ids.front())->second.node_ids;
        }
    }

    // === 步骤8: 回环检测和全局约束计算 ===
    // 对于每个已完成的子地图，检测与当前节点是否形成回环
    for(auto& elem : finished_submap_ids)
    {
        DetectLoopAndComputeConstraint(elem, node_id);
    }

    // === 注释掉的代码：更全面的回环检测策略 ===
    // 这部分代码实现了当子地图刚完成时，检测该子地图与所有历史节点的回环
    // if (newly_finished_submap) 
    // {
    //     const unsigned int newly_finished_submap_id = submap_ids.front();
    //     
    //     // 遍历所有历史节点
    //     for (const auto& node_id_data : optimization_problem_->node_data()) 
    //     {
    //         const unsigned int node_id = node_id_data.first;
    //         // 如果该节点不属于刚完成的子地图，尝试检测回环
    //         if (newly_finished_submap_node_ids.count(node_id) == 0) 
    //         {
    //             DetectLoopAndComputeConstraint(newly_finished_submap_id, node_id);
    //         }
    //     }
    // }

    // === 注释掉的代码：调试和优化触发 ===
    // ROS_INFO("constraints size: %d", data_.constraints.size());

    // === 注释掉的代码：定期优化触发 ===
    // 这部分实现了每90个节点后自动触发一次优化
    // num_nodes_since_last_loop_closure_++;
    // if (num_nodes_since_last_loop_closure_>90)
    // {
    //     RunOptimization();
    // }
}

void PoseGraph::RunOptimization()
{

    const TrajectoryNode::Data* constant_data;
    const Submap* submap;
    for(auto elem : submap_node_has_looped)
    {
        constant_data = data_.trajectory_nodes.at(elem.second).constant_data.get();
        submap = static_cast<const Submap*>(data_.submap_data.at(elem.first).submap.get());
        Eigen::Vector3d pose_estimated = Eigen::Vector3d::Zero();
        Eigen::Vector3d initial_pose = constant_data->local_pose;
 
        RealTimeCorrelativeScanMatcher real_csm;
        float score = 0;
  
        if(real_csm.Match(constant_data->local_pose,
                        constant_data->filtered_semantic_data,
                        *(submap->grid()),pose_estimated) > 0.7)
        {
            running_optimization = true;
    
            Eigen::Vector3d constraint_transform = ComputeRelativePose(submap->local_pose(), pose_estimated);
            data_.constraints.push_back(Constraint{elem.first, elem.second, 1.1e4, 1e5,
                                                constraint_transform, Constraint::Tag::Global});
        }
    }



    if (optimization_problem_->submap_data().empty())
    {
        return;
    }

    optimization_problem_->Solve(data_.constraints);
    
    num_nodes_since_last_loop_closure_ = 0;

    const auto& submap_data = optimization_problem_->submap_data();
    const auto& node_data = optimization_problem_->node_data();

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

    path_estimated_after_spa.header.frame_id = "world";
    path_estimated_after_spa_pub.publish(path_estimated_after_spa);
    data_.global_submap_poses_2d = submap_data;

    // update trajectory nodes
    Eigen::Vector3d local_to_global;
    for(const auto& node : node_data)
    {
        data_.trajectory_nodes.at(node.first).global_pose = node.second.global_pose_2d;
    }

    // Eigen::Matrix3d T_gloabl_local, T_global_node, T_local_node;
    
    // Eigen::Vector3d last_optimized_node_pose = std::prev(node_data.end())->second.global_pose_2d;
    // Eigen::Vector3d last_node_pose = std::prev(node_data.end())->second.local_pose_2d;

    // T_global_node << cos(last_optimized_node_pose[2]), -1*sin(last_optimized_node_pose[2]), last_optimized_node_pose[0],
    //                  sin(last_optimized_node_pose[2]), cos(last_optimized_node_pose[2]), last_optimized_node_pose[1],
    //                  0,0,1;

    // T_local_node << cos(last_node_pose[2]), -1*sin(last_node_pose[2]), last_node_pose[]

}


std::vector<unsigned int> PoseGraph::InitializeGlobalSubmapPoses(const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
{
    const auto& submap_data = optimization_problem_->submap_data();

    if (insertion_submaps.size() == 1)
    {
        if (submap_data.size()==0)
        {
            optimization_problem_->AddSubmap(insertion_submaps.front()->local_pose());
        }
        return {0};
    }
    
    unsigned int last_submap_id = std::prev(submap_data.end())->first;
    if (data_.submap_data.find(last_submap_id)->second.submap == insertion_submaps.front())
    {
        optimization_problem_->AddSubmap(insertion_submaps.back()->local_pose());
        return {last_submap_id, last_submap_id+1};
    }

    return {last_submap_id-1, last_submap_id};   
}


// 作用：计算当前“本地坐标系”到“全局坐标系”的变换矩阵（4x4 SE(3)齐次变换）
// 通常用于将当前帧/节点的位姿或点云从本地转换到全局地图坐标
Eigen::Matrix4d PoseGraph::ComputeLocalToGlobalTransform()
{
    // 1. 加锁，保证多线程环境下data_数据安全
    std::unique_lock<std::mutex> lock(mMutex_data_);

    // 2. 如果还没有全局子地图位姿（系统刚启动），直接返回单位矩阵
    if (data_.global_submap_poses.empty())
    {
        return Eigen::Matrix4d::Identity(); // 单位矩阵，无变换
    }

    // 3. 取出最新的（最后一个）全局子地图的位姿（编号+三维[x, y, yaw]）
    auto iter = data_.global_submap_poses.rbegin();
    
    // 4. 构造“全局→子地图”坐标系的齐次变换（T_global_submap）
    //    这里假设只有平面SE(2)位姿，z为0（二维位姿图）
    Eigen::Matrix4d T_global_submap;
    T_global_submap << 
        cos(iter->second[2]), -1*sin(iter->second[2]), 0, iter->second[0],
        sin(iter->second[2]),  cos(iter->second[2]), 0, iter->second[1],
        0,                    0,                    1, 0,
        0,                    0,                    0, 1;

    // 5. 找到当前子地图在本地坐标系下的原点（局部三维[x, y, yaw]）
    Eigen::Vector3d submap_local_pose = 
        data_.submap_data.find(iter->first)->second.submap->local_pose();

    // 6. 构造“本地→子地图”坐标系的齐次变换（T_local_submap）
    Eigen::Matrix4d T_local_submap;
    T_local_submap << 
        cos(submap_local_pose[2]), -1*sin(submap_local_pose[2]), 0, submap_local_pose[0],
        sin(submap_local_pose[2]),  cos(submap_local_pose[2]), 0, submap_local_pose[1],
        0,                         0,                        1, 0,
        0,                         0,                        0, 1;  

    // 7. 计算本地坐标系到全局坐标系的变换：T_global_submap * (T_local_submap)^(-1)
    //    （先从本地变到子地图，再从子地图变到全局，相当于本地→全局）
    Eigen::Matrix4d ret = T_global_submap * T_local_submap.inverse();

    // 8. 返回变换矩阵
    return ret;
}


std::map<unsigned int, TrajectoryNode> PoseGraph::getSubmapList()
{
    std::unique_lock<std::mutex> lock(mMutex_data_);
    return data_.trajectory_nodes;
}

} // namespace mapping
} // namespace AVP
