/**
 * @file gridmap.cc
 * @brief 2D概率栅格地图 - 子图的底层数据结构
 *
 * 栅格地图存储每个格子的 correspondence_cost（空闲代价），
 * 值越大表示该格子越可能是空闲的，值越小表示越可能被占据。
 *
 * 关键概念：
 *   - correspondence_cost: 空闲代价，与占据概率互为补数
 *     → 占据概率 = 1 - correspondence_cost
 *   - 值用 uint16 存储（经过查表压缩），节省内存
 *   - kUpdateMarker: 标记一轮更新中已被修改的格子，避免重复更新
 *
 * 坐标系约定：
 *   - max_ 为栅格地图右上角对应的世界坐标
 *   - cell_index(0) = row = y方向, cell_index(1) = col = x方向
 *   - 世界坐标 → 栅格索引: GetCellIndex()
 *   - 栅格索引 → 一维数组: ToFlatIndex()
 *
 * 动态扩展：
 *   当点落在地图范围外时，GrowLimits() 会将地图扩大为2倍
 *   （中心不变，向四周等比例扩展），并将旧数据复制到新地图中
 */

#include "gridmap.h"

namespace AVP
{
    
namespace mapping
{

/**
 * 构造函数：创建指定大小的栅格地图，所有格子初始化为未知状态
 */
GridMap::GridMap(const MapLimits& limits, ValueConversionTables* conversion_tables)
    : limits_(limits), conversion_tables_(conversion_tables),
    correspondence_cost_cells_(
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          kUnknownCorrespondenceValue)
{
    // 预计算 uint16值 → float空闲代价 的查找表
    value_to_correspondence_cost_table_ = conversion_tables->GetConversionTable(
          max_correspondence_cost_, min_correspondence_cost_,
          max_correspondence_cost_);
}

// 简化版 GrowLimits：只传入一个点
void GridMap::GrowLimits(const Eigen::Vector2f& point) {
  GrowLimits(point, {mutable_correspondence_cost_cells()},
             {kUnknownCorrespondenceValue});
}

/**
 * 动态扩展地图：当点落在当前地图范围外时，将地图扩大为2倍
 *
 * 扩展策略：
 *   - 地图中心不变，x/y方向各向两侧扩展一半
 *   - 新格子填充为 unknown 值
 *   - 旧数据复制到新地图的对应位置
 *   - 循环直到点位于地图范围内（极端情况可能扩展多次）
 */
void GridMap::GrowLimits(const Eigen::Vector2f& point,
                        const std::vector<std::vector<uint16>*>& grids,
                        const std::vector<uint16>& grids_unknown_cell_values) {

  while (!limits_.Contains(limits_.GetCellIndex(point))) {
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;

    // 新地图大小为原来的2倍，max坐标向右上偏移
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    // 旧地图数据在新地图一维数组中的起始偏移
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;

    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) {
      std::vector<uint16> new_cells(new_size,
                                    grids_unknown_cell_values[grid_index]);
      // 逐行复制旧地图数据到新地图中
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      *grids[grid_index] = new_cells;
    }

    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      // 已知区域的包围盒也需要平移
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

/**
 * 通过查找表更新一个格子的占据概率
 *
 * 查找表机制（来自Cartographer）：
 *   table[old_value] = new_value
 *   相当于贝叶斯更新的查表加速版本
 *
 * kUpdateMarker 机制：
 *   每个格子的值在一轮更新中会被加上 kUpdateMarker
 *   防止同一帧中对同一个格子重复更新
 *   FinishUpdate() 时会减去 kUpdateMarker 恢复正常值
 */
bool GridMap::ApplyLookupTable(const Eigen::Array2i& cell_index, const std::vector<uint16>& table)
{
    const int flat_index = ToFlatIndex(cell_index);
    uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];

    // 已被本轮更新过（值 >= kUpdateMarker），跳过
    if (*cell >= kUpdateMarker) {
        return false;
    }
    mutable_update_indices()->push_back(flat_index);

    // 通过查表实现贝叶斯概率更新
    *cell = table[*cell];

    // 扩展已知区域的包围盒
    mutable_known_cells_box()->extend(cell_index.matrix());
    return true;
}

// 一轮更新结束后，将所有被标记的格子减去 kUpdateMarker，恢复正常值域
void GridMap::FinishUpdate() 
{
  while (!update_indices_.empty()) 
  {
      correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
      update_indices_.pop_back();
  }
}

} // namespace mapping
} // namespace AVP
