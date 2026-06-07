/**
 * @file local_trajectory_builder.cc
 * @brief 局部轨迹构建器 - SLAM前端核心
 *
 * 核心职责：
 *   1. 接收里程计数据，用 PoseExtrapolator 预测当前位姿
 *   2. 累积语义扫描数据
 *   3. 用扫描匹配器（Ceres / 实时相关匹配）精细校正位姿
 *   4. 将匹配后的点云插入到子图中
 *
 * 核心数据流（每帧语义扫描触发）：
 *
 *   AddSemanticScan(scan)
 *     │
 *     ├── PoseExtrapolator::PredictPose()          // 里程计预测当前位姿
 *     ├── 将 scan 变换到世界坐标系 → 累积
 *     ├── 将累积点云变回车辆坐标系(tracking frame)
 *     │
 *     └── AddAccumulatedSemantics()
 *           ├── 体素滤波降采样
 *           ├── ScanMatch()                         // 扫描匹配精化位姿
 *           │     ├── [可选] RealTimeCorrelativeScanMatcher (暴力搜索)
 *           │     └── CeresScanMatcher2D            (非线性优化)
 *           │
 *           └── InsertIntoSubmap()                  // 将语义点云插入子图
 *                 ├── MotionFilter 过滤微小运动
 *                 └── ActiveSubmaps::InsertSemanticData
 */

#include "local_trajectory_builder.h"
#include "slam_file_logger.h"
namespace AVP
{
namespace mapping
{

LocalTrajectoryBuilder::LocalTrajectoryBuilder()
{
    // 发布累积的语义扫描点云（绿色，用于RViz可视化）
    accumulated_semantic_scan_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("accumulated_semantic_scan",0);
    // 发布扫描匹配后的估计轨迹
    estimated_path_pub_ = node_handle_.advertise<nav_msgs::Path>("path_estimated",0);
    // 发布原始带噪声的里程计轨迹（用于对比观察校正效果）
    noise_path_pub_ = node_handle_.advertise<nav_msgs::Path>("path_noise",0);
    path_estimated_.header.frame_id = "world";
    path_noise_.header.frame_id = "world";
}

/**
 * 处理一帧语义扫描数据（主入口函数）
 *
 * 步骤：
 *   1. 首帧时初始化 PoseExtrapolator
 *   2. 用里程计预测当前位姿
 *   3. 将语义点云变换到世界坐标系并累积
 *   4. 累积完成后，变换回车辆坐标系(tracking frame)
 *   5. 调用 AddAccumulatedSemantics 进行扫描匹配和子图插入
 */
std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> LocalTrajectoryBuilder::AddSemanticScan(
    const pcl::PointCloud<pcl::PointXYZ>& semantic_scan)
{
    // 第一帧数据到来时，初始化位姿外推器，以原点(0,0,0)为初始位姿
    if (extrapolator_==nullptr)
    {
        extrapolator_.reset(new PoseExtrapolator);
        extrapolator_->AddPose(Eigen::Vector3d::Zero());
        return nullptr;
    }
    
    if (num_accumulated == 0)
    {
        accumulated_semantic_data_.clear();
    }

    // 利用里程计增量预测当前位姿: last_scan_pose + (current_odom - last_odom)
    Eigen::Vector3d predict_pose = extrapolator_->PredictPose();
    
    pcl::PointCloud<pcl::PointXYZ> temp = semantic_scan;

    // 将语义点云从车辆坐标系变换到世界坐标系
    TransformPointCloud(temp,predict_pose);
    accumulated_semantic_data_ += temp;
    num_accumulated++;

    // 当前实现中 num_accumulated 每帧都立即处理（阈值 > 0 恒成立）
    // 如果需要多帧累积，可修改此阈值
    if (num_accumulated > 0)
    {   
        // 将累积的语义点云着色为绿色，发布用于RViz可视化
        pcl::PointCloud<pcl::PointXYZRGB> colored_accumulated_semantic_data;
        pcl::copyPointCloud(accumulated_semantic_data_, colored_accumulated_semantic_data);
        for(auto iter = colored_accumulated_semantic_data.begin(); iter != colored_accumulated_semantic_data.end(); iter++)
        {
            iter->r = 0; iter->g = 255; iter->b = 0;
        }

        sensor_msgs::PointCloud2 colored_accumulated_semantic_scan_pub_msg;
        pcl::toROSMsg(colored_accumulated_semantic_data, colored_accumulated_semantic_scan_pub_msg);
        colored_accumulated_semantic_scan_pub_msg.header.frame_id = "world";
        accumulated_semantic_scan_pub_.publish(colored_accumulated_semantic_scan_pub_msg);

        num_accumulated = 0;
        
        // 构造 世界坐标系→车辆坐标系 的变换矩阵 T_vehicle_w = T_w_vehicle^{-1}
        // 将累积点云变换回车辆坐标系(tracking frame)，因为扫描匹配需要在tracking frame下进行
        Eigen::Matrix4d T_w_vehicle;
        T_w_vehicle << cos(predict_pose[2]), -1*sin(predict_pose[2]), 0, predict_pose[0],
                    sin(predict_pose[2]), cos(predict_pose[2]), 0, predict_pose[1],
                    0,0,1,0,
                    0,0,0,1;
        Eigen::Matrix4d T_vehicle_w = T_w_vehicle.inverse();
        TransformPointCloud(accumulated_semantic_data_, T_vehicle_w);

        // 进入扫描匹配 + 子图插入流程
        return AddAccumulatedSemantics(accumulated_semantic_data_, predict_pose);
    }
    return nullptr;
}

// 接收里程计数据，更新位姿外推器，同时记录噪声轨迹
void LocalTrajectoryBuilder::AddOdometryData(const Eigen::Vector3d& odometry_pose)
{
    if (extrapolator_==nullptr)
        return;

    // 将里程计数据送入 PoseExtrapolator 的队列
    extrapolator_->AddOdometry(odometry_pose);

    // 记录原始噪声轨迹，用于和匹配后的轨迹对比
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = odometry_pose[0];
    pose_msg.pose.position.y = odometry_pose[1];
    pose_msg.pose.orientation.z = odometry_pose[2];
    path_noise_.poses.push_back(pose_msg);
}

/**
 * 处理累积的语义数据：体素滤波 → 扫描匹配 → 子图插入
 *
 * @param accumulated_semantics  累积的语义点云（tracking frame）
 * @param global_pose            预测的全局位姿（用作扫描匹配的初始值）
 */
std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> LocalTrajectoryBuilder::AddAccumulatedSemantics(const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantics,
                                                     Eigen::Vector3d& global_pose)
{
    // 体素滤波降采样，减少点云数量以加速扫描匹配
    pcl::PointCloud<pcl::PointXYZ> filtered_accumulated_semantics;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(accumulated_semantics.makeShared());
    filter.setLeafSize(0.1f, 0.1f, 0.1f);
    filter.filter(filtered_accumulated_semantics);

    // 扫描匹配：用预测位姿作为初值，与已有子图对齐，得到精化后的位姿
    std::shared_ptr<Eigen::Vector3d> pose_estimated = ScanMatch(global_pose, filtered_accumulated_semantics);
    if(pose_estimated == nullptr)
    {
        SlamFileLogger::Instance().Logf(
            "SCAN_MATCH",
            "failed predict_pose=(%.3f, %.3f, %.3f) points=%zu",
            global_pose[0], global_pose[1], global_pose[2], filtered_accumulated_semantics.size());
        return nullptr;
    }

    // 用匹配后的精确位姿，将语义点云变换到世界坐标系
    pcl::PointCloud<pcl::PointXYZ> accumulated_semantics_in_world = accumulated_semantics;
    TransformPointCloud(accumulated_semantics_in_world, *pose_estimated);

    // 将世界坐标系下的点云插入子图
    std::unique_ptr<InsertionResult> insertion_result =  
        InsertIntoSubmap(accumulated_semantics_in_world, filtered_accumulated_semantics, *pose_estimated);

    return std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>( new LocalTrajectoryBuilder::MatchingResult{
        global_pose, std::move(accumulated_semantics_in_world), std::move(insertion_result)
    });
}

/**
 * 扫描匹配：将当前语义点云与已有子图栅格对齐，精细校正位姿
 *
 * 匹配策略（两阶段，参考Cartographer）：
 *   1. [可选] RealTimeCorrelativeScanMatcher: 暴力穷举搜索，在一定角度/位移范围内找最优匹配
 *      → 适合初始误差较大的情况，但计算量大
 *   2. CeresScanMatcher2D: 基于Ceres非线性优化，利用双三次插值的栅格值做梯度下降
 *      → 精度高但需要较好的初值（由阶段1或里程计预测提供）
 *
 * @param predict_pose     预测位姿（x, y, yaw），作为匹配的初始值
 * @param downSampled_cloud 降采样后的点云（tracking frame）
 * @return 匹配后的精化位姿
 */
std::shared_ptr<Eigen::Vector3d> LocalTrajectoryBuilder::ScanMatch(const Eigen::Vector3d& predict_pose, 
                                       const pcl::PointCloud<pcl::PointXYZ>& downSampled_cloud)
{
    // 如果还没有子图，直接用预测位姿（第一帧数据）
    if (active_submaps_.submaps().empty())
    {
        Eigen::Vector3d pose_estimated = predict_pose;
        return std::make_shared<Eigen::Vector3d>(pose_estimated);
    }
    
    // 使用第一个活跃子图的栅格地图作为匹配目标
    std::shared_ptr<const Submap> matching_submap = active_submaps_.submaps().front();

    Eigen::Vector3d pose_estimated = predict_pose;

    // 阶段1 [可选]: 实时相关扫描匹配（暴力搜索），默认关闭
    if (use_real_time_correlative_scan_match)
    {
        real_time_correlative_scan_matcher_.Match(predict_pose, downSampled_cloud, 
                                                  *matching_submap->grid(), pose_estimated);
    }
    
    std::shared_ptr<Eigen::Vector3d> pose_observation(new Eigen::Vector3d);

    // 阶段2: Ceres扫描匹配（非线性优化），在子图栅格上做梯度下降
    // 残差项包含：占据空间代价（点落在free区域的惩罚）+ 平移约束
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(pose_estimated.head<2>(), 
                              pose_estimated, downSampled_cloud, 
                              *matching_submap->grid(), pose_observation.get(),
                              &summary);

    // 记录匹配后的位姿到轨迹中
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = (*pose_observation)[0];
    pose_msg.pose.position.y = (*pose_observation)[1];
    pose_msg.pose.orientation.z = (*pose_observation)[2];
    path_estimated_.poses.push_back(pose_msg);

    // 发布匹配后的轨迹和噪声轨迹（用于对比）
    estimated_path_pub_.publish(path_estimated_);
    noise_path_pub_.publish(path_noise_);

    extrapolator_->AddPose(*pose_observation);

    const Eigen::Vector3d delta = *pose_observation - predict_pose;
    SlamFileLogger::Instance().Logf(
        "SCAN_MATCH",
        "success predict=(%.3f,%.3f,%.3f) matched=(%.3f,%.3f,%.3f) delta=(%.3f,%.3f,%.3f) ceres_iters=%d points=%zu",
        predict_pose[0], predict_pose[1], predict_pose[2],
        (*pose_observation)[0], (*pose_observation)[1], (*pose_observation)[2],
        delta[0], delta[1], delta[2],
        static_cast<int>(summary.iterations.size()),
        downSampled_cloud.size());

    return pose_observation;
}

/**
 * 将匹配后的语义点云插入子图
 *
 * @param accumulated_semantic           世界坐标系下的完整语义点云
 * @param filtered_accumulated_semantic  降采样后的语义点云（存储在轨迹节点中）
 * @param pose_estimated                 扫描匹配后的位姿
 * @return InsertionResult 或 nullptr（如果运动太小被过滤掉）
 */
std::unique_ptr<LocalTrajectoryBuilder::InsertionResult> LocalTrajectoryBuilder::InsertIntoSubmap(
                                   const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantic,
                                   const pcl::PointCloud<pcl::PointXYZ>& filtered_accumulated_semantic,
                                   const Eigen::Vector3d& pose_estimated)
{
    // 运动过滤：如果位移<0.2m 且角度差的平方<0.2（即角度差<√0.2≈0.45rad≈26°），跳过插入
    if (motion_filter_.IsSimilar(pose_estimated))
    {
        SlamFileLogger::Instance().Logf(
            "INSERT",
            "skipped by motion filter pose=(%.3f, %.3f, %.3f)",
            pose_estimated[0], pose_estimated[1], pose_estimated[2]);
        return nullptr;
    }

    // 将语义点云插入活跃子图（同时更新概率栅格地图）
    std::vector<std::shared_ptr<const Submap>> insertion_submaps = 
        active_submaps_.InsertSemanticData(accumulated_semantic, pose_estimated);

    // 构造 InsertionResult，包含轨迹节点数据和关联的子图
    // 这些信息会被传递给 PoseGraph::AddNode
    SlamFileLogger::Instance().Logf(
        "INSERT",
        "inserted pose=(%.3f, %.3f, %.3f) submaps=%zu points=%zu",
        pose_estimated[0], pose_estimated[1], pose_estimated[2],
        insertion_submaps.size(), filtered_accumulated_semantic.size());

    return std::unique_ptr<InsertionResult>(new InsertionResult{
        std::make_shared<TrajectoryNode::Data>(TrajectoryNode::Data{
            filtered_accumulated_semantic,
            pose_estimated
        }),
        std::move(insertion_submaps)
    });
}

} // namespace mapping
} // namespace AVP
