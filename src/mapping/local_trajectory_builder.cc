#include "local_trajectory_builder.h"
namespace AVP
{
namespace mapping
{

// LocalTrajectoryBuilder类的构造函数实现
LocalTrajectoryBuilder::LocalTrajectoryBuilder()
{
    // 1. 创建一个用于发布累计语义点云的ROS话题
    // 发布类型为sensor_msgs::PointCloud2，话题名为"accumulated_semantic_scan"
    // 队列长度为0，表示异步发布（一般适用于只发布最新数据的场景）
    accumulated_semantic_scan_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("accumulated_semantic_scan", 0);

    // 2. 创建一个用于发布估计轨迹的ROS话题
    // 发布类型为nav_msgs::Path，话题名为"path_estimated"
    estimated_path_pub_ = node_handle_.advertise<nav_msgs::Path>("path_estimated", 0);

    // 3. 创建一个用于发布带有噪声轨迹的ROS话题
    // 发布类型为nav_msgs::Path，话题名为"path_noise"
    noise_path_pub_ = node_handle_.advertise<nav_msgs::Path>("path_noise", 0);

    // 4. 初始化成员变量path_estimated_和path_noise_的坐标系frame_id
    // "world"表示这些路径信息以世界坐标系为基准
    path_estimated_.header.frame_id = "world";
    path_noise_.header.frame_id = "world";
}

/**
 * @brief 添加语义扫描数据到轨迹构建器
 * @param semantic_scan 输入的语义点云扫描数据
 * @return 匹配结果，包含位姿估计和插入结果，如果处理失败则返回nullptr
 * 
 * 这是SLAM前端的核心函数，处理流程：
 * 1. 初始化位姿外推器（如果是第一次）
 * 2. 预测当前位姿
 * 3. 将点云变换到世界坐标系并累积
 * 4. 当累积足够数据后，执行扫描匹配和子地图插入
 */
std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> LocalTrajectoryBuilder::AddSemanticScan(
    const pcl::PointCloud<pcl::PointXYZ>& semantic_scan)
{
    // 第一次调用时初始化位姿外推器
    if (extrapolator_==nullptr)
    {
        extrapolator_.reset(new PoseExtrapolator);
        extrapolator_->AddPose(Eigen::Vector3d::Zero());  // 添加零位姿作为初始位姿
        return nullptr;  // 初始化完成，等待下次调用
    }
    
    // 如果是新的累积周期，清空之前的累积数据
    if (num_accumulated == 0)
    {
        accumulated_semantic_data_.clear();
    }
    
    // 使用位姿外推器预测当前机器人位姿
    Eigen::Vector3d predict_pose = extrapolator_->PredictPose();
    
    // 复制输入点云到临时变量，避免修改原始数据
    pcl::PointCloud<pcl::PointXYZ> temp = semantic_scan;

    // 将点云从机器人坐标系变换到世界坐标系
    TransformPointCloud(temp,predict_pose);
    
    // 累积变换后的点云数据
    accumulated_semantic_data_ += temp;
    num_accumulated++;  // 累积计数器加1

    // 当有累积数据时，执行可视化发布和进一步处理
    if (num_accumulated > 0)
    {   
        // === 可视化部分：为累积点云添加颜色并发布 ===
        pcl::PointCloud<pcl::PointXYZRGB> colored_accumulated_semantic_data;
        pcl::copyPointCloud(accumulated_semantic_data_, colored_accumulated_semantic_data);
        
        // 将所有点染成绿色 (R=0, G=255, B=0)
        for(auto iter = colored_accumulated_semantic_data.begin(); iter != colored_accumulated_semantic_data.end(); iter++)
        {
            iter->r = 0; iter->g = 255; iter->b = 0;
        }

        // 转换为ROS消息并发布，供RViz可视化
        sensor_msgs::PointCloud2 colored_accumulated_semantic_scan_pub_msg;
        pcl::toROSMsg(colored_accumulated_semantic_data, colored_accumulated_semantic_scan_pub_msg);
        colored_accumulated_semantic_scan_pub_msg.header.frame_id = "world";
        accumulated_semantic_scan_pub_.publish(colored_accumulated_semantic_scan_pub_msg);

        // 重置累积计数器，准备下一轮累积
        num_accumulated = 0;
        
        // === 坐标变换：将累积数据转换回机器人坐标系 ===
        // 构建从世界坐标系到机器人坐标系的变换矩阵
        Eigen::Matrix4d T_w_vehicle;  // 世界到机器人的变换矩阵
        T_w_vehicle << cos(predict_pose[2]), -1*sin(predict_pose[2]), 0, predict_pose[0],
                    sin(predict_pose[2]), cos(predict_pose[2]), 0, predict_pose[1],
                    0,0,1,0,
                    0,0,0,1;
        Eigen::Matrix4d T_vehicle_w = T_w_vehicle.inverse();  // 机器人到世界的逆变换
        
        // 将累积点云从世界坐标系变换回机器人坐标系（跟踪坐标系）
        TransformPointCloud(accumulated_semantic_data_, T_vehicle_w);

        // 调用累积语义数据处理函数，执行扫描匹配和子地图插入
        return AddAccumulatedSemantics(accumulated_semantic_data_, predict_pose);
    }
    
    // 如果还没有足够的累积数据，返回nullptr等待更多数据
    return nullptr;
}

void LocalTrajectoryBuilder::AddOdometryData(const Eigen::Vector3d& odometry_pose)
{
    if (extrapolator_==nullptr)
        return;

    extrapolator_->AddOdometry(odometry_pose);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = odometry_pose[0];
    pose_msg.pose.position.y = odometry_pose[1];
    pose_msg.pose.orientation.z = odometry_pose[2];
    path_noise_.poses.push_back(pose_msg);
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> 
    LocalTrajectoryBuilder::AddAccumulatedSemantics(
    const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantics,
                                                     Eigen::Vector3d& global_pose)
{
    // 1. 对输入的累积语义点云做体素滤波（VoxelGrid滤波），降采样，提高匹配效率
    pcl::PointCloud<pcl::PointXYZ> filtered_accumulated_semantics;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(accumulated_semantics.makeShared());
    filter.setLeafSize(0.1f, 0.1f, 0.1f);
    // makeShared() 调用后会返回一个指向该点云对象的 boost::shared_ptr 智能指针
    filter.filter(filtered_accumulated_semantics);
    // 2. 用滤波后的点云做扫描匹配，估计当前位姿
    // pose_estimated为扫描匹配后输出的估计位姿（指针，可能匹配失败）
    std::shared_ptr<Eigen::Vector3d> pose_estimated = ScanMatch(global_pose, filtered_accumulated_semantics);
    if(pose_estimated == nullptr)
    {
        ROS_WARN("Scan Matching failed");
        return nullptr;
    }
    // 3. 将原始累积点云（未滤波）用新估计的位姿变换到世界坐标系下
    pcl::PointCloud<pcl::PointXYZ> accumulated_semantics_in_world = accumulated_semantics;
    TransformPointCloud(accumulated_semantics_in_world, *pose_estimated);
    // 4. 将点云插入到子地图（通常是用于地图拼接、后端优化等）
    std::unique_ptr<InsertionResult> insertion_result =  
        InsertIntoSubmap(accumulated_semantics_in_world, filtered_accumulated_semantics, *pose_estimated);
    // 5. 构造匹配结果结构体，返回智能指针
    return std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>( new LocalTrajectoryBuilder::MatchingResult{
        global_pose, std::move(accumulated_semantics_in_world), std::move(insertion_result)
    });
}

// 作用：对输入的下采样点云做扫描匹配，估计当前帧的位姿（相对于子地图）
// 参数：
//   predict_pose         ：预测位姿（比如由运动模型或里程计推算）
//   downSampled_cloud    ：下采样后的点云（当前帧）
// 返回值：
//   指向匹配后位姿的智能指针（Eigen::Vector3d）

std::shared_ptr<Eigen::Vector3d> LocalTrajectoryBuilder::ScanMatch(
    const Eigen::Vector3d& predict_pose, 
    const pcl::PointCloud<pcl::PointXYZ>& downSampled_cloud)
{
    // 1. 如果当前没有可用的子地图（比如刚开始建图时），直接返回预测位姿
    if (active_submaps_.submaps().empty())
    {
        Eigen::Vector3d pose_estimated = predict_pose;
        return std::make_shared<Eigen::Vector3d>(pose_estimated);
    }
    
    // 2. 取出当前需要匹配的子地图（一般为活动子地图队列的第一个）
    std::shared_ptr<const Submap> matching_submap = active_submaps_.submaps().front();

    // 3. 初始化位姿为预测位姿
    Eigen::Vector3d pose_estimated = predict_pose;

    // 4. 如果启用了实时相关性扫描匹配（通常用于全局粗匹配/大范围搜索）
    if (use_real_time_correlative_scan_match)
    {
        // 先用相关性扫描匹配粗配，得到更好的初值
        real_time_correlative_scan_matcher_.Match(
            predict_pose, 
            downSampled_cloud, 
            *matching_submap->grid(), 
            pose_estimated
        );
    }
    
    // 5. 构造一个指针用于保存最终精确匹配后的位姿
    std::shared_ptr<Eigen::Vector3d> pose_observation(new Eigen::Vector3d);

    // 6. 用ceres优化器进行精细扫描匹配（以相关性结果或预测位姿为初值）
    //    输入：初始平面位姿（前2维）、全位姿（3维）、当前点云、子地图栅格、输出位姿指针、优化器运行信息
    ceres::Solver::Summary summary;
    ceres_scan_matcher_.Match(
        pose_estimated.head<2>(),    // x, y（初值）
        pose_estimated,              // [x, y, yaw]（初值）
        downSampled_cloud, 
        *matching_submap->grid(), 
        pose_observation.get(),      // 输出结果存入这个地址
        &summary                     // ceres优化报告
    );

    // 7. 构造ROS消息，将优化后位姿加入轨迹
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = (*pose_observation)[0];
    pose_msg.pose.position.y = (*pose_observation)[1];
    pose_msg.pose.orientation.z = (*pose_observation)[2];  // yaw存储在z，这里只做示例，实际姿态需转换
    path_estimated_.poses.push_back(pose_msg);

    // 8. 发布估计轨迹和噪声轨迹（用于可视化/调试）
    estimated_path_pub_.publish(path_estimated_);
    noise_path_pub_.publish(path_noise_);

    // 9. 将当前位姿结果加入外推器（用于后续时间同步、预测等）
    extrapolator_->AddPose(*pose_observation);

    // 10. 返回最终估计的位姿
    return pose_observation;
}

// 作用：将新采集到的语义点云数据插入当前子地图（submap），并返回插入结果
// 参数：
//   accumulated_semantic           ：原始累积语义点云（未滤波，用于地图融合）
//   filtered_accumulated_semantic  ：滤波后的点云（用于节点存储和后端优化）
//   pose_estimated                 ：点云在全局下的估计位姿
// 返回值：
//   指向插入结果的唯一智能指针（InsertionResult），如不满足运动阈值，则返回nullptr

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult> LocalTrajectoryBuilder::InsertIntoSubmap(
                                   const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantic,
                                   const pcl::PointCloud<pcl::PointXYZ>& filtered_accumulated_semantic,
                                   const Eigen::Vector3d& pose_estimated)
{
    // 1. 检查运动是否足够大（避免频繁插入重复数据）
    //    若当前位姿与上次插入的位姿相似，则不插入，直接返回空指针
    if (motion_filter_.IsSimilar(pose_estimated))
    {
        return nullptr;
    }
    // 2. 将语义点云和位姿插入当前活跃子地图，并返回所有被插入的子地图指针集合
    std::vector<std::shared_ptr<const Submap>> insertion_submaps = 
        active_submaps_.InsertSemanticData(accumulated_semantic, pose_estimated);


    auto node_data = std::make_shared<TrajectoryNode::Data>(
        filtered_accumulated_semantic, pose_estimated);

    auto insertion_result = std::unique_ptr<InsertionResult>(
        new InsertionResult{node_data, std::move(insertion_submaps)}
    );

    return insertion_result;
}

} // namespace mapping
} // namespace AVP
