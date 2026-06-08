#ifndef TRAJECTORY_NODE_H
#define TRAJECTORY_NODE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace AVP
{
namespace mapping
{

struct TrajectoryNode
{
    struct Data
    {
        Data() = default;
        Data(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
             const Eigen::Vector3d& pose)
            : filtered_semantic_data(semantic_data), local_pose(pose) {}

        pcl::PointCloud<pcl::PointXYZ> filtered_semantic_data;
        Eigen::Vector3d local_pose;
    };

    std::shared_ptr<const Data> constant_data;
    Eigen::Vector3d global_pose;
};

} // namespace mapping
} // namespace AVP


#endif