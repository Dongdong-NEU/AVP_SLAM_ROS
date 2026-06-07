/**
 * @file ceres_scan_matcher_2d.cc
 * @brief Ceres扫描匹配器 - 基于非线性优化的精细位姿校正
 *
 * 算法原理：
 *   将扫描匹配问题建模为非线性最小二乘优化问题，使用 Ceres Solver 求解。
 *   优化变量: pose = (x, y, θ)
 *
 *   残差项（代价函数）：
 *   1. OccupiedSpaceCostFunction2D（占据空间代价）：
 *      → 将点云中每个点按当前位姿变换到世界坐标系
 *      → 查询栅格地图中对应位置的 free 值（correspondence cost）
 *      → free 值越大说明该位置越可能是空闲的 → 残差越大
 *      → 目标：让所有点都落在占据概率高的区域（free值小）
 *      → 使用双三次插值(BiCubicInterpolator)使代价函数可微
 *
 *   2. TranslationDeltaCostFunctor2D（平移约束）：
 *      → 惩罚匹配后的平移与目标平移的偏差
 *      → 防止优化结果偏离预测位姿太远
 *
 *   3. [注释掉] RotationDeltaCostFunctor2D（旋转约束）：
 *      → 如果启用，会约束旋转角度不变
 *
 * 相比暴力搜索的优势：利用梯度下降，计算量小，精度高
 * 前提条件：需要较好的初值（由里程计预测或暴力搜索提供）
 */

#include "ceres_scan_matcher_2d.h"

namespace AVP
{
    
namespace mapping
{
    
/**
 * @param target_translation    目标平移量（通常是预测的平移，用于平移约束）
 * @param initial_pose_estimate 位姿初值 (x, y, θ)
 * @param point_cloud           tracking frame 下的语义点云
 * @param grid                  用于匹配的子图栅格地图
 * @param pose_estimate         [输出] 优化后的位姿
 * @param summary               [输出] Ceres求解器的运行摘要
 */
void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const Eigen::Vector3d& initial_pose_estimate,
                               const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
                               const GridMap& grid,
                               Eigen::Vector3d* const pose_estimate,
                               ceres::Solver::Summary* const summary) const {
  // 优化变量：(x, y, θ)，以预测位姿作为初始值
  double ceres_pose_estimate[3] = {initial_pose_estimate[0],
                                   initial_pose_estimate[1],
                                   initial_pose_estimate[2]};
  ceres::Problem problem;
  ceres::Solver::Options ceres_solver_options_;
  ceres_solver_options_.max_num_iterations = 50;

  // 残差1: 占据空间代价 — 核心残差项
  // scaling_factor = 1/√N，使残差与点云大小无关
  // 每个点产生一个残差维度，总残差维度 = 点云大小（动态残差）
    problem.AddResidualBlock(
        CreateOccupiedSpaceCostFunction2D(
            1. /
                std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid),
        nullptr, ceres_pose_estimate);

  // 残差2: 平移约束 — 防止优化结果偏离预测位姿太远
  // 权重 = 8，target_translation = 预测的平移分量
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          8, target_translation),
      nullptr, ceres_pose_estimate);

  // 残差3 [已禁用]: 旋转约束 — 固定角度不变
  // 如果启用，优化器只调整平移，角度保持预测值
  // problem.AddResidualBlock(
  //     RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
  //         1, ceres_pose_estimate[2]),
  //     nullptr, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = Eigen::Vector3d
      {ceres_pose_estimate[0], ceres_pose_estimate[1], ceres_pose_estimate[2]};
}

} // namespace mapping


} // namespace AVP
