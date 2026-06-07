/**
 * @file global_trajectory_builder.cc
 * @brief 全局轨迹构建器 - SLAM系统的数据总调度中心
 */

#include "global_trajectory_builder.h"
#include "slam_file_logger.h"

#include <ros/package.h>
#include <pcl/io/pcd_io.h>

namespace AVP
{
namespace mapping
{

    GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(PoseGraph* const pose_graph, std::shared_ptr<LocalTrajectoryBuilder> local_trajectory_builder)
    :pose_graph_{pose_graph}, local_trajectory_builder_{local_trajectory_builder}
    {
        ros::NodeHandle private_nh("~");
        SlamFileLogger::Instance().Init(private_nh);

        const std::string default_output_dir = ros::package::getPath("avp") + "/output";
        private_nh.param<std::string>("map_save_path", map_save_path_, default_output_dir + "/semantic_map_final.pcd");
        private_nh.param<bool>("save_map_on_shutdown", save_map_on_shutdown_, true);
        private_nh.param<bool>("save_map_each_optimization", save_map_each_optimization_, true);

        const size_t dot_pos = map_save_path_.find_last_of('.');
        if (dot_pos != std::string::npos)
        {
            map_save_latest_path_ = map_save_path_.substr(0, dot_pos) + "_latest" + map_save_path_.substr(dot_pos);
        }
        else
        {
            map_save_latest_path_ = map_save_path_ + "_latest";
        }

        odometry_sub_ = node_handle_.subscribe("odometry_noised",0,&GlobalTrajectoryBuilder::odometry_callback,this);
        semantic_scan_sub_ = node_handle_.subscribe("scan_semantic_points",0,&GlobalTrajectoryBuilder::semantic_callback,this);
        semantic_map_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("semantic_map",0);
        markers_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("markers",0);

        timers_.push_back(node_handle_.createTimer(
            ros::Duration(5), &GlobalTrajectoryBuilder::PubSemanticMap,this));

        SlamFileLogger::Instance().Log("INIT", "GlobalTrajectoryBuilder started");
        SlamFileLogger::Instance().Logf("INIT", "map_save_path=%s", map_save_path_.c_str());
        SlamFileLogger::Instance().Logf("INIT", "map_save_latest_path=%s", map_save_latest_path_.c_str());
    }

    GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder()
    {
        OnShutdown();
    }

    void GlobalTrajectoryBuilder::AddSensorData(const Eigen::Vector3d& odometery_pose)
    {
        local_trajectory_builder_->AddOdometryData(odometery_pose);
    }

    void GlobalTrajectoryBuilder::AddSensorData(const pcl::PointCloud<pcl::PointXYZ>& semantic_scan)
    {
        std::unique_ptr<LocalTrajectoryBuilder::MatchingResult> matching_result =
            local_trajectory_builder_->AddSemanticScan(semantic_scan);

        if (matching_result == nullptr)
            return;

        if (matching_result->insertion_result!=nullptr)
        {
            pose_graph_->AddNode(matching_result->insertion_result->constant_data,
                                 matching_result->insertion_result->insertion_submaps);
        }
    }

    void GlobalTrajectoryBuilder::odometry_callback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        Eigen::Vector3d odometry_pose;
        odometry_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z;
        AddSensorData(odometry_pose);
    }

    void GlobalTrajectoryBuilder::semantic_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ> semantic_scan;

        pcl::fromROSMsg(*msg, semantic_scan);
        AddSensorData(semantic_scan);
    }

    void GlobalTrajectoryBuilder::SaveSemanticMapToFile(
        const pcl::PointCloud<pcl::PointXYZRGB>& map_cloud, const std::string& reason)
    {
        if (map_cloud.empty())
        {
            SlamFileLogger::Instance().Logf("MAP_SAVE", "skip (%s): empty point cloud", reason.c_str());
            return;
        }

        const std::string& target_path =
            (reason == "shutdown_final") ? map_save_path_ : map_save_latest_path_;

        if (!SlamFileLogger::EnsureParentDirectoryForFile(target_path))
        {
            SlamFileLogger::Instance().Logf(
                "MAP_SAVE", "failed to create parent directory for %s", target_path.c_str());
            return;
        }

        if (pcl::io::savePCDFileBinary(target_path, map_cloud) == 0)
        {
            SlamFileLogger::Instance().Logf(
                "MAP_SAVE", "%s saved %zu points to %s",
                reason.c_str(), map_cloud.size(), target_path.c_str());
        }
        else
        {
            SlamFileLogger::Instance().Logf(
                "MAP_SAVE", "failed to save %s to %s", reason.c_str(), target_path.c_str());
        }
    }

    void GlobalTrajectoryBuilder::OnShutdown()
    {
        SlamFileLogger::Instance().Log("SHUTDOWN", "mapping node shutting down");

        if (!save_map_on_shutdown_ || !has_last_semantic_map_)
        {
            if (!has_last_semantic_map_)
            {
                SlamFileLogger::Instance().Log("MAP_SAVE", "skip shutdown save: no map built yet");
            }
            return;
        }

        SaveSemanticMapToFile(last_semantic_map_, "shutdown_final");
    }

    void GlobalTrajectoryBuilder::PubSemanticMap(const ros::TimerEvent& timer_event)
    {
        SlamFileLogger::Instance().Log("OPTIMIZE", "trigger periodic global optimization and map publish");

        const size_t constraints_before = pose_graph_->GetConstraintCount();
        const size_t nodes_before = pose_graph_->GetNodeCount();
        const size_t submaps_before = pose_graph_->GetSubmapCount();

        pose_graph_->RunOptimization();

        const size_t constraints_after = pose_graph_->GetConstraintCount();
        const size_t global_constraints = pose_graph_->GetGlobalConstraintCount();

        SlamFileLogger::Instance().Logf(
            "OPTIMIZE",
            "finished: nodes=%zu submaps=%zu constraints %zu->%zu (global=%zu)",
            nodes_before, submaps_before, constraints_before, constraints_after, global_constraints);

        pcl::PointCloud<pcl::PointXYZRGB> accumulated_submap_;
        visualization_msgs::MarkerArray marker_array_;

        std::map<unsigned int, TrajectoryNode> all_submap_list = pose_graph_->getSubmapList();

        for(auto it = all_submap_list.begin(); it != all_submap_list.end(); it++)
        {
            pcl::PointCloud<pcl::PointXYZ> temp = it->second.constant_data->filtered_semantic_data;

            Eigen::Vector3d pose_temp = it->second.global_pose;
            TransformPointCloud(temp, pose_temp);

            pcl::PointCloud<pcl::PointXYZRGB> submap;
            pcl::copyPointCloud(temp, submap);

            for(size_t i = 0; i < submap.points.size(); ++i)
            {
                submap.points[i].r = 0;
                submap.points[i].g = 255;
                submap.points[i].b = 0;
            }

            accumulated_submap_ += submap;
        }

        pcl::PointCloud<pcl::PointXYZRGB> filtered_accumulated_submap;
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(accumulated_submap_.makeShared());
        filter.setLeafSize(0.1f, 0.1f, 0.1f);
        filter.filter(filtered_accumulated_submap);

        sensor_msgs::PointCloud2 semantic_map_msg;
        pcl::toROSMsg(accumulated_submap_, semantic_map_msg);
        semantic_map_msg.header.frame_id = "world";
        semantic_map_pub_.publish(semantic_map_msg);

        last_semantic_map_ = accumulated_submap_;
        has_last_semantic_map_ = true;

        SlamFileLogger::Instance().Logf(
            "MAP_PUB", "published /semantic_map with %zu points (filtered=%zu)",
            accumulated_submap_.size(), filtered_accumulated_submap.size());

        if (save_map_each_optimization_)
        {
            SaveSemanticMapToFile(accumulated_submap_, "optimization_latest");
        }
    }


} // namespace mapping
} // namespace AVP
