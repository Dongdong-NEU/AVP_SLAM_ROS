/**
 * @file mapping_node_main.cc
 * @brief AVP语义SLAM系统的主入口节点
 *
 * 整体架构（参考Cartographer设计）：
 *
 *   ROS话题输入:
 *     /odometry_noised      (带噪声的里程计)
 *     /scan_semantic_points  (语义点云，如车位线、箭头等)
 *         │
 *         ▼
 *   GlobalTrajectoryBuilder  ← 总调度器，订阅ROS话题，分发数据
 *         │
 *         ├──► LocalTrajectoryBuilder  ← 局部建图：扫描匹配 + 子图插入
 *         │       ├── PoseExtrapolator    (用里程计预测当前位姿)
 *         │       ├── ScanMatch           (用Ceres/相关匹配器精细校正位姿)
 *         │       └── ActiveSubmaps       (管理当前活跃的子图)
 *         │
 *         └──► PoseGraph  ← 全局优化：回环检测 + SPA位姿图优化
 *                 ├── DetectLoop           (距离阈值检测潜在回环)
 *                 ├── Constraint           (子图-节点之间的约束)
 *                 └── OptimizationProblem  (Ceres SPA求解)
 *
 *   ROS话题输出:
 *     /semantic_map                 (全局语义地图点云)
 *     /path_estimated               (扫描匹配后的轨迹)
 *     /path_estimated_after_spa     (全局优化后的轨迹)
 *     /path_noise                   (原始带噪声的轨迹)
 */

#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <boost/bind.hpp>
#include "pose_extrapolator.h"
#include "global_trajectory_builder.h"
#include "pose_graph.h"
#include "local_trajectory_builder.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    
    ros::start();

    // 1. 创建局部轨迹构建器：负责扫描匹配和子图管理
    std::shared_ptr<AVP::mapping::LocalTrajectoryBuilder> local_trajectory_builder(
        new AVP::mapping::LocalTrajectoryBuilder
    );
 
    // 2. 创建全局轨迹构建器：内部持有 PoseGraph，订阅ROS话题并将数据分发给局部建图和位姿图
    //    构造函数中会自动订阅 /odometry_noised 和 /scan_semantic_points
    AVP::mapping::GlobalTrajectoryBuilder global_trajectory_builder2D(
        new AVP::mapping::PoseGraph, local_trajectory_builder);
    
    // 3. 进入ROS事件循环，等待话题回调触发SLAM流程
    ros::spin();
    

    return 0;
}