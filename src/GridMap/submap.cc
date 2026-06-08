#include "submap.h"

namespace AVP
{
namespace mapping
{
    
    // Submap类的构造函数
    // 作用：用指定的原点位姿、栅格地图和查表工具创建一个子地图对象
    // 参数：
    //   local_pose         —— 子地图原点（在本地/全局坐标系下的三维位姿）
    //   grid              —— 该子地图的概率栅格地图对象，使用unique_ptr进行所有权转移
    //   conversion_tables —— 概率/代价值转换用查找表指针

    Submap::Submap(
        const Eigen::Vector3d& local_pose, 
        std::unique_ptr<GridMap> grid, 
        ValueConversionTables* conversion_tables
    ) 
        : local_pose_(local_pose),                     // 1. 初始化子地图原点位姿
        conversion_tables_(conversion_tables)        // 2. 保存查表工具指针
    {
        grid_ = std::move(grid);                       // 3. 转移GridMap对象的所有权给成员变量grid_
    }

    // 作用：将一帧语义点云数据插入当前子地图（Submap），并更新点云集合与计数
    // 参数：
    //   semantic_data —— 本帧语义点云数据
    //   inserter      —— 地图插入器，负责将点云写入概率栅格地图

    void Submap::InsertSemanticData(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
                                    ProbabilityGridRangeDataInserter* inserter)
    {
        // 1. 调用插入器，把当前点云数据写入概率栅格地图（做概率/占据概率等更新）
        inserter->Insert(semantic_data, grid_.get());

        // 2. 把当前点云数据累加到本子地图的数据集中（用于后续回环、重建、可视化等）
        data_ += semantic_data;

        // 3. 更新“累计点云帧数”计数器（用于判断何时需要生成/关闭子地图）
        set_num_accumulated_semantic_data(num_accumulated_semantic_data() + 1);
    }
    // 作用：将一帧新的语义点云数据插入所有活跃子地图，并根据累积数量控制子地图的创建与“封存”状态。
    // 参数：
    //   semantic_data  ：当前帧语义点云
    //   pose_estimated ：点云对应的位姿
    // 返回值：
    //   当前所有活跃子地图的指针集合
    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::InsertSemanticData(
        const pcl::PointCloud<pcl::PointXYZ>& semantic_data, const Eigen::Vector3d& pose_estimated)
    {
        // 1. 如果当前没有子地图，或最新子地图已达到最大点云帧数，则新建一个子地图
        if (submaps_.empty() || submaps_.back()->num_accumulated_semantic_data() == max_semantic_data_in_submap)
        {
            // 以当前位姿为中心创建新子地图
            AddSubmap(pose_estimated);
        }
        // 2. 将当前点云插入所有活跃子地图（一般来说活跃子地图数目不多，通常为2）
        for(auto elem : submaps_)
        {
            elem->InsertSemanticData(semantic_data, range_data_inserter.get());
        }
        // 3. 如果最早的子地图已经累积了2倍最大点云帧数，认为其建图已完成，将其标记为“插入完成”
        if (submaps_.front()->num_accumulated_semantic_data() == 2*max_semantic_data_in_submap)
        {
            submaps_.front()->set_insertion_finished(true);
        }
        // 4. 返回当前所有活跃子地图的指针集合（便于后续节点/地图管理用）
        return submaps();
    }

    // 作用：以指定原点origin创建一个新的子地图（Submap），并管理活跃子地图的数量（滑动窗口管理）
    // 参数：origin —— 新子地图的中心/起始位姿
    void ActiveSubmaps::AddSubmap(const Eigen::Vector3d& origin)
    {
        // 1. 如果当前活跃子地图数量>=2（一般设定滑窗最大数为2），则删除最旧的子地图，保证内存占用受控
        if (submaps_.size()>=2)
        {
            // 删除队首（最早的）子地图
            // 保证活跃子地图数量不超过2，限制内存使用
            submaps_.erase(submaps_.begin());
        }
        // 2. 创建一个新的子地图，并放入submaps_队列尾部
        submaps_.push_back(std::unique_ptr<Submap>(
            // 新子地图原点
            new Submap{origin, 
                 // 创建并转移一个GridMap指针，.release() 把unique_ptr的所有权交给Submap，避免双重管理
                std::unique_ptr<GridMap>(CreateGrid(origin).release()),
                // 格式转换表（用于概率/栅格值等查表操作）
         &conversion_tables_}));
    }

    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::submaps() const
    {
        return std::vector<std::shared_ptr<const Submap>>(submaps_.begin(),submaps_.end());
    }

    // 作用：以指定原点 origin 创建一个指定分辨率和尺寸的栅格地图（GridMap）对象
    // 参数：origin —— 新子地图的起点（一般为机器人当前位置）
    // 返回值：新创建的GridMap的唯一智能指针

    std::unique_ptr<GridMap> ActiveSubmaps::CreateGrid(const Eigen::Vector3d& origin)
    {
        constexpr int kInitialSubmapSize = 100;  // 1. 子地图的初始尺寸（单位：格子数，正方形）

        float resolution = 0.05;                 // 2. 地图分辨率（每格大小，单位：米/格）

        // 3. 计算地图的限制参数 MapLimits
        //    - resolution：每格的物理尺寸
        //    - origin.head<2>() + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones()       // 地图的原点（右上角），以当前origin的xy加上地图半径距离
        //    - CellLimits：地图的格子数量（x、y方向）
        MapLimits limit{
            resolution, 
            // “以origin为中心，往右上角平移半个子地图的距离”，用作地图的原点坐标（一般是右上角或左下角，根据实际地图实现）
            origin.head<2>().cast<double>() + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
            // 这里的 CellLimits 用于指定地图有多少格子（X方向/Y方向各多少格）
            CellLimits(kInitialSubmapSize, kInitialSubmapSize)
        };

        // 4. 用上述参数新建一个GridMap，并用unique_ptr管理返回
        return std::unique_ptr<GridMap>(
            new GridMap(limit, &conversion_tables_)
        );      
    }




} // namespace mapping
} // namespace AVP
