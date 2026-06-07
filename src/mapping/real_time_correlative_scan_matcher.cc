/**
 * @file real_time_correlative_scan_matcher.cc
 * @brief 实时相关扫描匹配器 - 暴力穷举搜索最优位姿
 *
 * 算法原理（参考Cartographer论文）：
 *   在给定的搜索窗口内（平移 ± 线性窗口，旋转 ± 角度窗口），
 *   穷举所有可能的 (dx, dy, dθ) 组合，
 *   将点云投影到栅格地图上，计算匹配得分（占据概率的均值），
 *   选择得分最高的候选解作为匹配结果。
 *
 * 搜索流程：
 *   1. 按角度搜索窗口生成多个旋转角度的点云副本
 *   2. 将旋转后的点云离散化到栅格坐标系
 *   3. 穷举所有 (角度索引, x偏移, y偏移) 组合
 *   4. 对每个候选解，计算点云各点在栅格上的占据概率均值
 *   5. 用距离和角度的指数衰减对得分加权（偏好小位移）
 *   6. 返回最高得分的候选解
 *
 * 优点：不依赖初值，全局搜索能力强
 * 缺点：计算量大，候选解数量 = num_scans × num_x × num_y
 */

#include "real_time_correlative_scan_matcher.h"

namespace AVP
{

namespace mapping
{

    /**
     * 计算单个候选解的得分
     *
     * 将点云中的每个点加上候选的 (x,y) 偏移后，查询栅格地图中对应位置的占据概率，
     * 最终返回所有点的平均概率作为得分。得分越高表示点云与地图越匹配。
     */
    float ComputeCandidateScore(const GridMap& probability_grid,
                                const DiscreteScan2D& discrete_scan,
                                int x_index_offset, int y_index_offset) {
    float candidate_score = 0.f;
    for (const Eigen::Array2i& xy_index : discrete_scan) {
        // 对每个点的栅格索引加上候选的偏移量（相当于平移点云）
        const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                            xy_index.y() + y_index_offset);
        // 查询栅格中该位置的占据概率（从 correspondence_cost 转换）
        const float probability = CorrespondenceCostToProbability(
            probability_grid.GetCorrespondenceCost(proposed_xy_index)
        );
        candidate_score += probability;
    }
    // 归一化：平均占据概率
    candidate_score /= static_cast<float>(discrete_scan.size());
    return candidate_score;
    }

    RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher()
    {
        image_transport::ImageTransport it(node_handle_);
        grid_map_image_pub_ = it.advertise("submap_grid_map_image",1);
    }


    /**
     * 执行实时相关扫描匹配
     *
     * @param predict_pose                 预测的位姿 (x, y, yaw)
     * @param semantics_in_tracking_frame  车辆坐标系(tracking frame)下的语义点云
     * @param grid                         用于匹配的子图栅格地图
     * @param pose_estimated               [输出] 匹配后的位姿
     * @return 最佳候选解的得分 (0~1)
     */
    double RealTimeCorrelativeScanMatcher::Match(const Eigen::Vector3d& predict_pose, 
                                                 const pcl::PointCloud<pcl::PointXYZ>& semantics_in_tracking_frame,                             
                                                 const GridMap& grid, Eigen::Vector3d& pose_estimated)
    {
        // Step 1: 将 tracking frame 下的点云应用预测的旋转角度
        // 使角度搜索以 0 为中心进行，后续只搜索小角度扰动
        pcl::PointCloud<pcl::PointXYZ> rotated_point_cloud = 
            TransformPointCloudAndReturn(semantics_in_tracking_frame, Eigen::Vector3d{0,0,predict_pose[2]});

        // Step 2: 定义搜索空间
        // 线性搜索窗口 = 5m, 角度搜索窗口 = π/6 ≈ 30°
        const SearchParameters search_parameters(5, M_PI / 6, rotated_point_cloud, grid.limits().resolution());

        // Step 3: 按角度步长生成多个旋转角度的点云副本
        const std::vector<pcl::PointCloud<pcl::PointXYZ>> rotated_scans =
            GenerateRotatedScans(rotated_point_cloud, search_parameters);

        // Step 4: 将旋转后的点云离散化到栅格坐标系（加上预测的平移量）
        const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
                                    grid.limits(), rotated_scans,
                                    Eigen::Translation2f(predict_pose[0],predict_pose[1])
                                    );

        // Step 5: 生成所有候选解（角度索引 × x偏移 × y偏移 的笛卡尔积）
        std::vector<Candidate2D> candidates = GenerateExhaustiveSearchCandidates(search_parameters);

        // Step 6: 对每个候选解评分（查询栅格 + 距离加权）
        ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

        // Step 7: 选择最高得分的候选解
        const Candidate2D& best_candidate = *std::max_element(candidates.begin(), candidates.end());

        // 将最优偏移量叠加到预测位姿上，得到匹配后的位姿
        pose_estimated[0] = predict_pose[0] + best_candidate.x;
        pose_estimated[1] = predict_pose[1] + best_candidate.y;
        pose_estimated[2] = predict_pose[2] + best_candidate.orientation;

        return best_candidate.score;

    }

    /**
     * 生成穷举搜索的所有候选解
     *
     * 候选解数量 = Σ(每个旋转角度下的 x候选数 × y候选数)
     * 对于当前参数：约 num_scans × (2×num_linear+1)²
     */
    std::vector<Candidate2D> RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchCandidates(
        const SearchParameters& search_parameters) const {
    int num_candidates = 0;
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        const int num_linear_x_candidates =
            (search_parameters.linear_bounds[scan_index].max_x -
            search_parameters.linear_bounds[scan_index].min_x + 1);
        const int num_linear_y_candidates =
            (search_parameters.linear_bounds[scan_index].max_y -
            search_parameters.linear_bounds[scan_index].min_y + 1);
        num_candidates += num_linear_x_candidates * num_linear_y_candidates;
    }

    std::vector<Candidate2D> candidates;
    candidates.reserve(num_candidates);

    // 三层嵌套循环：角度 × x偏移 × y偏移
    for (int scan_index = 0; scan_index != search_parameters.num_scans;
        ++scan_index) {
        for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
            x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
            ++x_index_offset) {
        for (int y_index_offset =
                search_parameters.linear_bounds[scan_index].min_y;
            y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
            ++y_index_offset) {
            candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                    search_parameters);
        }
        }
    }
    return candidates;
    }

    /**
     * 对所有候选解评分
     *
     * 评分 = 栅格匹配得分 × 距离衰减权重
     * 距离衰减: exp(-(√(dx²+dy²) × 0.1 + |dθ| × 0.1)²)
     * → 偏好位移小、角度变化小的候选解（越靠近预测位姿权重越高）
     */
    void RealTimeCorrelativeScanMatcher::ScoreCandidates(
        const GridMap& grid, const std::vector<DiscreteScan2D>& discrete_scans,
        const SearchParameters& search_parameters,
        std::vector<Candidate2D>* const candidates) const 
    {
        for (Candidate2D& candidate : *candidates)
        {
            // 计算栅格匹配得分（点云各点占据概率的均值）
            candidate.score = ComputeCandidateScore(
                static_cast<const GridMap&>(grid),
                discrete_scans[candidate.scan_index], candidate.x_index_offset,
                candidate.y_index_offset);

            // 乘以距离衰减因子：偏离预测位姿越远，得分越低
            candidate.score *=
                std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                        0.1 +
                                    std::abs(candidate.orientation) *
                                        0.1));
        }      
    }
}
}