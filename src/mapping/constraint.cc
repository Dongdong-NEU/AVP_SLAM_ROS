/**
 * @file constraint.cc
 * @brief 约束计算 - 子图与节点之间的相对位姿
 *
 * 约束表示一条"观测"：节点 j 在子图 i 的坐标系下的相对位姿。
 *
 * 两种约束类型：
 *   - Non_Global (局部约束)：节点与其所属子图之间的约束，来源于扫描匹配结果
 *     权重较小 (500, 1600)
 *   - Global (全局约束)：回环检测产生的约束，连接较远的子图和节点
 *     权重较大 (11000, 100000)，强制拉近回环处的位姿
 */

#include "constraint.h"

namespace AVP
{
namespace mapping
{

/**
 * 计算节点相对于子图的相对位姿 T_submap^{-1} * T_node
 *
 * 数学推导：
 *   设子图全局位姿为 (sx, sy, sθ)，节点全局位姿为 (nx, ny, nθ)
 *   
 *   1. 先求全局坐标系下的平移差:
 *      Δx = nx - sx,  Δy = ny - sy
 *   
 *   2. 将平移差旋转到子图坐标系（乘以 R_submap^T）：
 *      [dx]   [cos(sθ)  sin(sθ)] [Δx]
 *      [dy] = [-sin(sθ) cos(sθ)] [Δy]
 *   
 *   3. 角度差:
 *      Δθ = normalize(nθ - sθ)
 *
 * @return 节点在子图坐标系下的相对位姿 (dx, dy, dθ)
 */
Eigen::Vector3d ComputeRelativePose(const Eigen::Vector3d& submap_global_pose, const Eigen::Vector3d& node_global_pose)
{
    double tx = node_global_pose[0] - submap_global_pose[0];
    double ty = node_global_pose[1] - submap_global_pose[1];
    double r = normalize(node_global_pose[2] - submap_global_pose[2]);

    double cz = cos(submap_global_pose[2]);
    double sz = sin(submap_global_pose[2]);

    // R_submap^T * [tx, ty]^T → 子图坐标系下的相对平移
    return {cz*tx+sz*ty,
            -1*sz*tx+cz*ty,
            r};
}


} // namespace mapping
} // namespace AVP
