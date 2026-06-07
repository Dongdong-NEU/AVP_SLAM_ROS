/**
 * @file occupied_space_cost_function.cc
 * @brief 占据空间代价函数 - Ceres扫描匹配的核心残差定义
 *
 * 这是 CeresScanMatcher2D 中最重要的残差项。
 *
 * 原理：
 *   对于点云中的每个点，按当前优化的位姿 (x,y,θ) 变换到世界坐标系，
 *   然后在栅格地图中查询该位置的空闲代价 (correspondence cost)。
 *
 *   如果点落在占据区域 → 空闲代价小 → 残差小 → 好
 *   如果点落在空闲区域 → 空闲代价大 → 残差大 → 差
 *
 *   Ceres优化器会调整 (x,y,θ) 使总残差最小，即让点云尽量落在占据区域。
 *
 * 技术要点：
 *   - 使用 Ceres::BiCubicInterpolator 对栅格值做双三次插值
 *     → 使代价函数连续可微，Ceres可以用自动微分计算梯度
 *   - GridArrayAdapter 在地图边界外填充最大空闲代价 (kPadding)
 *   - 残差维度 = 点云大小（动态残差），每个点产生一个残差值
 */

#include "occupied_space_cost_function.h"

namespace AVP
{
    
namespace mapping
{
    
class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
                              const GridMap& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = grid_.limits();

    for (size_t i = 0; i < point_cloud_.points.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_.points[i].x)),
                                         (T(point_cloud_.points[i].y)),
                                         T(1.));
      // 根据预测位姿对单个点进行坐标变换
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      // 获取三次插值之后的栅格free值, Evaluate函数内部调用了GetValue函数
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      // free值越小, 表示占用的概率越大
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  
  // 自定义网格
  class GridArrayAdapter {
   public:
    // 枚举 DATA_DIMENSION 表示被插值的向量或者函数的维度
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const GridMap& grid) : grid_(grid) {}

    // 获取栅格free值
    void GetValue(const int row, const int column, double* const value) const {
      // 处于地图外部时, 赋予最大free值
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } 
      // 根据索引获取free值
      else {
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    // map上下左右各增加 kPadding
    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const GridMap& grid_;
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const pcl::PointCloud<pcl::PointXYZ>& point_cloud_;
  const GridMap& grid_;
};



// 工厂函数, 返回地图的CostFunction
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
    const GridMap& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size()); // 比固定残差维度的 多了一个参数
}

} // namespace mapping
} // namespace AVP
