/**
 * @file submap.cc
 * @brief 子图管理 - 维护活跃子图和语义数据插入
 *
 * 子图生命周期（参考Cartographer设计）：
 *
 *   ActiveSubmaps 始终维护最多2个子图：[front=旧子图, back=新子图]
 *
 *   1. 系统启动时创建第一个子图
 *   2. 当新子图(back)累积了 max_semantic_data_in_submap (=20) 帧数据后，
 *      创建新子图并推入后端，原来的back变为front
 *   3. 每帧语义数据会同时插入到所有活跃子图中
 *   4. 当旧子图(front)累积了 2×max (=40) 帧后，标记为 insertion_finished
 *   5. 已完成的子图不再接收新数据，但会被用于扫描匹配和回环检测
 *
 *   时间线示意（第 N 次调用 InsertSemanticData）：
 *   第 1~20次:  [Submap_0]            ← 只有一个子图
 *   第 21次:    [Submap_0, Submap_1]  ← back累积满20帧，创建新子图
 *   第 21~39次: 同时写入两个子图
 *   第 40次:    Submap_0 累积满40帧，标记 insertion_finished
 *   第 41次:    Submap_1 累积满20帧 → 创建 Submap_2，移除 Submap_0
 *              [Submap_0, Submap_1] → [Submap_1, Submap_2]
 *   ...
 *
 *   这种重叠设计保证子图之间有连续性，便于回环检测
 */

#include "submap.h"

namespace AVP
{
namespace mapping
{
    
    Submap::Submap(const Eigen::Vector3d& local_pose, std::unique_ptr<GridMap> grid, 
            ValueConversionTables* conversion_tables):local_pose_(local_pose),conversion_tables_(conversion_tables)
    {
        grid_ = std::move(grid);
    }

    // 将语义点云插入子图：更新概率栅格 + 累积原始点云
    void Submap::InsertSemanticData(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
                                    ProbabilityGridRangeDataInserter* inserter)
    {
        // 通过 inserter 将点云写入概率栅格（更新占据概率）
        inserter->Insert(semantic_data, grid_.get());

        // 累积原始点云数据（用于后续的回环检测扫描匹配）
        data_ += semantic_data;
        set_num_accumulated_semantic_data(num_accumulated_semantic_data()+1);
    }

    /**
     * 向活跃子图中插入语义数据
     *
     * @param semantic_data   世界坐标系下的语义点云
     * @param pose_estimated  当前匹配后的位姿（用作新子图的原点）
     * @return 当前活跃子图列表
     */
    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::InsertSemanticData(
        const pcl::PointCloud<pcl::PointXYZ>& semantic_data, const Eigen::Vector3d& pose_estimated)
    {
        // 当没有子图或最新子图已满（达到 max_semantic_data_in_submap=20 帧），创建新子图
        if (submaps_.empty() || submaps_.back()->num_accumulated_semantic_data() == max_semantic_data_in_submap)
        {
            AddSubmap(pose_estimated);
        }

        // 将数据同时插入所有活跃子图（1~2个）
        for(auto elem : submaps_)
        {
            elem->InsertSemanticData(semantic_data, range_data_inserter.get());
        }

        // 旧子图(front)累积了 2×max=40 帧数据后，标记为完成
        // 完成后的子图将用于回环检测，不再接收新数据
        if (submaps_.front()->num_accumulated_semantic_data() == 2*max_semantic_data_in_submap)
        {
            submaps_.front()->set_insertion_finished(true);
        }

        return submaps();
    }

    // 添加新子图，保持最多2个活跃子图
    void ActiveSubmaps::AddSubmap(const Eigen::Vector3d& origin)
    {
        // 如果已有2个子图，移除最旧的（front）
        if (submaps_.size()>=2)
        {
            submaps_.erase(submaps_.begin());
        }
        // 以当前位姿为原点，创建新的栅格地图和子图
        submaps_.push_back(std::unique_ptr<Submap>(new Submap{origin, std::unique_ptr<GridMap>(CreateGrid(origin).release()),
         &conversion_tables_}));
    }

    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::submaps() const
    {
        return std::vector<std::shared_ptr<const Submap>>(submaps_.begin(),submaps_.end());
    }

    /**
     * 创建新的栅格地图
     *
     * 初始大小: 100×100 格, 分辨率 0.05m/格
     * 实际覆盖范围: 5m × 5m（以 origin 为中心）
     * 栅格地图会根据需要自动扩展（见 GridMap::GrowLimits）
     */
    std::unique_ptr<GridMap> ActiveSubmaps::CreateGrid(const Eigen::Vector3d& origin)
    {
        constexpr int kInitialSubmapSize = 100;
        float resolution = 0.05;
      
        // max 为栅格地图右上角的世界坐标（origin + 半幅宽度）
        MapLimits limit{resolution, 
                    origin.head<2>().cast<double>() + 0.5 * kInitialSubmapSize *resolution * Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)};

        return std::unique_ptr<GridMap>(
            new GridMap(limit, &conversion_tables_)
            );      
    }



} // namespace mapping
} // namespace AVP
