/**
 * @file optimization_problem.cc
 * @brief SPA全局优化问题 - 基于Ceres的稀疏位姿调整
 *
 * SPA (Sparse Pose Adjustment) 原理：
 *   将所有子图和轨迹节点的全局位姿作为优化变量，
 *   所有约束（局部约束 + 回环约束）作为残差项，
 *   通过 Ceres 非线性最小二乘求解最优的全局位姿分布。
 *
 * 优化变量：
 *   - 每个子图: (x, y, θ)_global
 *   - 每个节点: (x, y, θ)_global
 *
 * 约束（残差项）：
 *   对于每条约束 (submap_i, node_j, relative_pose)，残差定义为：
 *     error = 观测到的相对位姿 - 当前变量计算的相对位姿
 *     当前计算: h = R_submap^T × (T_node - T_submap)
 *     观测: constraint.relative_pose（建立约束时记录的值）
 *     加权后: scaled_error = (translation_weight × Δt, rotation_weight × Δr)
 *
 * 锚点：第一个子图位姿固定不变（SetParameterBlockConstant），防止自由度退化
 */

#include "optimization_problem.h"
#include "slam_file_logger.h"

namespace AVP
{
    
namespace mapping
{

// Eigen向量 → std::array（Ceres优化变量需要原始数组）
std::array<double, 3> FromPose(const Eigen::Vector3d& pose) 
{
  return {{pose[0], pose[1], normalize(pose[2])}};
}

// std::array → Eigen向量
Eigen::Vector3d ToPose(const std::array<double, 3>& values) 
{
  return Eigen::Vector3d{values[0], values[1], values[2]};
}

/**
 * 求解 SPA 优化问题
 *
 * 步骤：
 *   1. 为每个子图和节点创建优化变量（3维: x, y, θ）
 *   2. 固定第一个子图作为锚点
 *   3. 添加所有约束对应的残差项
 *   4. 调用 Ceres 求解
 *   5. 将优化结果写回 submap_data_ 和 node_data_
 */
void OptimizationProblem::Solve(std::vector<Constraint>& constraints)
{
    if (node_data_.empty())
    {
        return;
    }
    
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // 优化变量容器（用 std::array<double,3> 存储，方便传给Ceres）
    std::map<unsigned int, std::array<double, 3>> C_submaps;
    std::map<unsigned int, std::array<double, 3>> C_nodes;
    bool first_submap = true;

    // 注册所有子图的位姿为优化变量
    for(const auto& elem : submap_data_)
    {
        C_submaps.insert(std::make_pair(elem.first, FromPose(elem.second.global_pose)));
        problem.AddParameterBlock(C_submaps.at(elem.first).data(),3);
        // 第一个子图固定不动（作为全局坐标系的锚点）
        if (first_submap)
        {
            problem.SetParameterBlockConstant(C_submaps.at(elem.first).data());
            first_submap = false;
        }
    }

    // 注册所有节点的位姿为优化变量
    // 注意：每次优化都从 local_pose_2d（扫描匹配的原始结果）出发，
    // 而不是从上次优化后的 global_pose_2d 出发。
    // 这样设计使得每次 SPA 都基于原始观测重新求解全局最优。
    // 优化后的结果写入 global_pose_2d。
    for(const auto& elem : node_data_)
    {
        C_nodes.insert(std::make_pair(elem.first, FromPose(elem.second.local_pose_2d)));
        problem.AddParameterBlock(C_nodes.at(elem.first).data(),3);
        if (first_submap)
        {
            problem.SetParameterBlockConstant(C_nodes.at(elem.first).data());
            first_submap = false;
        }
    }

    // 将每条约束添加为残差项
    // SpaCostFunction2D 计算: 观测到的相对位姿与当前变量推算的相对位姿之间的加权误差
    for (const Constraint& constraint : constraints) 
    {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(constraint),
            nullptr,
            C_submaps.at(constraint.submap_id).data(),
            C_nodes.at(constraint.node_id).data()
        );
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    SlamFileLogger::Instance().Logf(
        "SPA",
        "Ceres %s initial_cost=%.6f final_cost=%.6f iterations=%d constraints=%zu submaps=%zu nodes=%zu",
        summary.termination_type == ceres::CONVERGENCE ? "CONVERGED" : "NOT_CONVERGED",
        summary.initial_cost, summary.final_cost,
        static_cast<int>(summary.iterations.size()),
        constraints.size(), submap_data_.size(), node_data_.size());

    // 将优化结果写回内部数据
    for(const auto& C_submap_id_data : C_submaps)
    {
        submap_data_.at(C_submap_id_data.first).global_pose = ToPose(C_submap_id_data.second);
    }
    for(const auto& C_node_id_data : C_nodes)
    {
        node_data_.at(C_node_id_data.first).global_pose_2d = ToPose(C_node_id_data.second);
    }
}

} // namespace mapping


} // namespace AVP
