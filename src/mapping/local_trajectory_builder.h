#ifndef LOCAL_TRAJECTORY_BUILDER_H
#define LOCAL_TRAJECTORY_BUILDER_H

// 局部轨迹构建器头文件
// 负责处理传感器数据并构建局部轨迹，是SLAM系统的核心组件之一

// ROS相关头文件
#include "pose_extrapolator.h"           // 位姿外推器，用于预测下一时刻的位姿
#include "sensor_msgs/PointCloud2.h"     // ROS点云消息类型
#include "nav_msgs/OccupancyGrid.h"      // 占用栅格地图消息类型
#include "nav_msgs/Path.h"               // 路径消息类型
#include <memory>                        // 智能指针
#include <ros/ros.h>                     // ROS核心库

// PCL点云处理库
#include <pcl/point_cloud.h>             // PCL点云基础类
#include <pcl/point_types.h>             // PCL点类型定义
#include <pcl_conversions/pcl_conversions.h> // PCL与ROS消息转换
#include <pcl/common/transforms.h>       // 点云变换
#include <pcl/filters/voxel_grid.h>      // 体素滤波器
#include <unistd.h>                      // UNIX标准定义

// 自定义模块头文件
#include "transform.h"                   // 坐标变换工具
#include "motion_filter.h"               // 运动滤波器
#include "trajectory_node.h"             // 轨迹节点定义
#include "submap.h"                      // 子地图
#include "real_time_correlative_scan_matcher.h" // 实时相关扫描匹配器
#include "ceres_scan_matcher_2d.h"       // Ceres优化扫描匹配器
namespace AVP
{
namespace mapping
{
    
/**
 * @class LocalTrajectoryBuilder
 * @brief 局部轨迹构建器类
 * 
 * 这是SLAM系统的核心组件之一，负责：
 * 1. 处理传感器数据（激光雷达点云、里程计）
 * 2. 执行扫描匹配来估计机器人位姿
 * 3. 管理局部子地图的构建和更新
 * 4. 提供轨迹节点数据给后端优化
 */
class LocalTrajectoryBuilder
{
public:

    LocalTrajectoryBuilder();

    /**
     * @struct InsertionResult
     * @brief 插入结果结构体
     * 
     * 当传感器数据被成功插入到子地图中时返回的结果
     */
    struct InsertionResult 
    {
        std::shared_ptr<const TrajectoryNode::Data> constant_data;  // 轨迹节点的常量数据
        std::vector<std::shared_ptr<const Submap>> insertion_submaps; // 插入数据的子地图列表
    };

    /**
     * @struct MatchingResult
     * @brief 扫描匹配结果结构体
     * 
     * 包含扫描匹配后的位姿估计和相关数据
     */
    struct MatchingResult 
    {
        Eigen::Vector3d local_pose;                                   // 局部坐标系下的位姿(x, y, theta)
        pcl::PointCloud<pcl::PointXYZ> semantic_data;               // 语义点云数据
        std::unique_ptr<const InsertionResult> insertion_result;     // 插入结果（如果执行了插入操作）
    };

    /**
     * @brief 添加语义扫描数据
     * @param semantic_scan 输入的语义点云扫描数据
     * @return 匹配结果，包含估计位姿和处理后的点云数据
     * 
     * 处理单次扫描的语义点云数据，执行扫描匹配获得位姿估计
     */
    std::unique_ptr<MatchingResult> AddSemanticScan(const pcl::PointCloud<pcl::PointXYZ>& semantic_scan);
    
    /**
     * @brief 添加里程计数据
     * @param odometry_pose 里程计位姿数据 (x, y, theta)
     * 
     * 更新位姿外推器，为下次扫描匹配提供初始位姿预测
     */
    void AddOdometryData(const Eigen::Vector3d& odometry_pose);

    /**
     * @brief 添加累积的语义数据
     * @param accumulated_semantics 累积的语义点云数据
     * @param global_pose 全局位姿估计
     * @return 匹配结果
     * 
     * 处理累积的语义点云数据，通常用于处理多帧累积后的数据
     */
    std::unique_ptr<MatchingResult> AddAccumulatedSemantics(const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantics, Eigen::Vector3d& global_pose);

    /**
     * @brief 执行扫描匹配
     * @param predict_pose 预测位姿
     * @param downSampled_cloud 降采样后的点云
     * @return 优化后的位姿指针
     * 
     * 使用预测位姿作为初值，通过扫描匹配优化得到更准确的位姿估计
     */
    std::shared_ptr<Eigen::Vector3d> ScanMatch(const Eigen::Vector3d& predict_pose, 
                                               const pcl::PointCloud<pcl::PointXYZ>& downSampled_cloud);

    /**
     * @brief 将数据插入到子地图中
     * @param accumulated_semantic 累积的语义点云
     * @param filtered_accumulated_semantic 滤波后的累积语义点云
     * @param pose_estimated 估计的位姿
     * @return 插入结果
     * 
     * 将处理后的点云数据和位姿信息插入到活跃子地图中
     */
    std::unique_ptr<InsertionResult> InsertIntoSubmap(  
                                   const pcl::PointCloud<pcl::PointXYZ>& accumulated_semantic,
                                   const pcl::PointCloud<pcl::PointXYZ>& filtered_accumulated_semantic,
                                   const Eigen::Vector3d& pose_estimated);

private:
    // 配置参数
    bool use_real_time_correlative_scan_match = false;  // 是否使用实时相关扫描匹配

    // ROS相关成员
    ros::NodeHandle node_handle_;                        // ROS节点句柄
    ros::Publisher accumulated_semantic_scan_pub_;      // 累积语义扫描发布器
    ros::Publisher estimated_path_pub_;                 // 估计路径发布器
    ros::Publisher noise_path_pub_;                     // 噪声路径发布器

    // 路径数据
    nav_msgs::Path path_estimated_;                     // 估计的路径
    nav_msgs::Path path_noise_;                         // 包含噪声的路径

    // 核心算法组件
    MotionFilter motion_filter_;                        // 运动滤波器，过滤微小运动
    std::unique_ptr<PoseExtrapolator> extrapolator_;   // 位姿外推器，预测下一时刻位姿
    
    // 数据累积相关
    int num_accumulated = 0;                            // 累积的扫描数量
    pcl::PointCloud<pcl::PointXYZ> accumulated_semantic_data_; // 累积的语义点云数据
    
    // 子地图管理
    ActiveSubmaps active_submaps_;                      // 活跃子地图管理器

    // 扫描匹配器
    RealTimeCorrelativeScanMatcher real_time_correlative_scan_matcher_; // 实时相关扫描匹配器
    CeresScanMatcher2D ceres_scan_matcher_;            // Ceres优化扫描匹配器

}; // class LocalTrajectoryBuilder

} // namespace mapping

} // namespace AVP

#endif // LOCAL_TRAJECTORY_BUILDER_H