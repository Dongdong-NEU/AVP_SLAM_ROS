# 分支差异分析：feature/learn vs feature/learning_dell

## 问题描述

- `feature/learn`：建图异常，子图没有被正确添加到地图中
- `feature/learning_dell`：建图正常

## 差异文件概览

共 42 个文件有差异，大部分为注释/格式改动。逻辑层面关键差异如下：

| 文件 | 差异类型 |
|------|---------|
| CMakeLists.txt | C++标准设置冲突 |
| trajectory_node.h | 构造函数差异 |
| local_trajectory_builder.cc | 日志系统 + Data 构造方式 |
| global_trajectory_builder.cc/h | 增加地图保存/日志功能 |
| pose_graph.cc | 回环验证算法 + running_optimization 管理 |
| slam_file_logger.cc/h | feature/learn 独有的日志系统 |

---

## 核心差异详解

### 1. `running_optimization` 标志管理 — 最可能的 Bug 根因

#### feature/learn 的实现

```cpp
void PoseGraph::RunOptimization()
{
    running_optimization = true;   // 一开始就设为 true
    
    // 遍历回环候选，使用 FastCSM 验证
    for (auto elem : submap_node_has_looped)
    {
        // 这些 .at() 调用如果 key 不存在会抛 std::out_of_range 异常
        const Eigen::Vector3d initial_relative_pose = ComputeRelativePose(
            optimization_problem_->submap_data().at(elem.first).global_pose,
            optimization_problem_->node_data().at(elem.second).global_pose_2d);
        
        FastCorrelativeScanMatcher2D fast_csm(*(submap->grid()));
        // ...
    }
    
    optimization_problem_->Solve(data_.constraints);
    // ...
    running_optimization = false;  // 最后才设回 false
}
```

#### feature/learning_dell 的实现

```cpp
void PoseGraph::RunOptimization()
{
    // 不在开头设置 running_optimization
    
    for (auto elem : submap_node_has_looped)
    {
        RealTimeCorrelativeScanMatcher real_csm;
        if(real_csm.Match(...) > 0.7)
        {
            running_optimization = true;  // 只在匹配成功时设置
            data_.constraints.push_back(...);
        }
    }
    
    optimization_problem_->Solve(data_.constraints);
    // running_optimization 不重置（后续影响较小）
}
```

#### 为什么这是关键

`ComputeConstraintsForNode()` 的入口有一个 guard：

```cpp
void PoseGraph::ComputeConstraintsForNode(...)
{
    if (running_optimization == true)
    {
        return;  // 跳过所有约束计算和子图注册
    }
    // ... InitializeGlobalSubmapPoses、AddTrajectoryNode、建约束、回环检测 ...
}
```

**故障链**：
1. Timer 触发 `RunOptimization()`
2. `running_optimization = true`
3. `.at(key)` 找不到 key → 抛出 `std::out_of_range`
4. `running_optimization = false` 永远不会执行
5. 后续所有 `ComputeConstraintsForNode` 调用被跳过
6. 新子图不注册、新节点不入优化 → 地图停止增长

---

### 2. 回环验证算法差异

| 维度 | feature/learn | feature/learning_dell |
|------|--------------|----------------------|
| 匹配器 | `FastCorrelativeScanMatcher2D`（分支定界） | `RealTimeCorrelativeScanMatcher`（暴力搜索） |
| 分数阈值 | 0.75 | 0.7 |
| 额外验证 | 有（`translation_deviation > 8m` 则拒绝） | 无 |
| 计算复杂度 | 高（多分辨率金字塔） | 中等 |
| 出错概率 | 较高（更多 .at() 调用） | 较低 |

---

### 3. CMakeLists.txt 编译标准

```cmake
# feature/learn（存在冲突）
add_compile_options(-std=c++11)   # 显式的 C++11 标志
set(CMAKE_CXX_STANDARD 14)       # CMake 管理的 C++14
# 两者可能冲突，最终编译标准取决于 GCC 命令行上哪个 -std= 参数在后面

# feature/learning_dell（干净）
##add_compile_options(-std=c++11)  # 已注释
set(CMAKE_CXX_STANDARD 14)        # 唯一标准设置
set(CMAKE_BUILD_TYPE Release)      # Release 优化
```

---

### 4. TrajectoryNode::Data 构造方式

#### feature/learn（无显式构造函数，使用聚合初始化）

```cpp
// trajectory_node.h
struct Data {
    pcl::PointCloud<pcl::PointXYZ> filtered_semantic_data;
    Eigen::Vector3d local_pose;
};

// local_trajectory_builder.cc
std::make_shared<TrajectoryNode::Data>(TrajectoryNode::Data{
    filtered_accumulated_semantic, pose_estimated})
```

#### feature/learning_dell（有显式构造函数）

```cpp
// trajectory_node.h
struct Data {
    Data() = default;
    Data(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
         const Eigen::Vector3d& pose)
        : filtered_semantic_data(semantic_data), local_pose(pose) {}
    
    pcl::PointCloud<pcl::PointXYZ> filtered_semantic_data;
    Eigen::Vector3d local_pose;
};

// local_trajectory_builder.cc
auto node_data = std::make_shared<TrajectoryNode::Data>(
    filtered_accumulated_semantic, pose_estimated);
```

---

### 5. SlamFileLogger 日志系统

`feature/learn` 独有，遍布于：
- `global_trajectory_builder.cc`（Init、地图保存、优化日志）
- `local_trajectory_builder.cc`（扫描匹配结果、插入日志）
- `pose_graph.cc`（节点添加、回环检测、优化过程）

功能：将 SLAM 过程详细写入文件 `output/logs/mapping_slam.log`。

本身不应影响逻辑，但引入了额外复杂度和初始化依赖。

---

### 6. GlobalTrajectoryBuilder 额外功能

`feature/learn` 独有：
- `SaveSemanticMapToFile()`：每次优化后保存 PCD 文件
- `OnShutdown()`：节点退出时保存最终地图
- 析构函数 `~GlobalTrajectoryBuilder()`
- ROS 参数：`map_save_path`、`save_map_on_shutdown`、`save_map_each_optimization`

---

## 修复建议

### 方案一：异常安全保护（最小改动）

```cpp
void PoseGraph::RunOptimization()
{
    running_optimization = true;
    try {
        // ... 原有逻辑 ...
    } catch (const std::exception& e) {
        ROS_ERROR("RunOptimization exception: %s", e.what());
    }
    running_optimization = false;  // 确保一定执行
}
```

### 方案二：参照 feature/learning_dell 重构标志管理

不在 `RunOptimization` 开头设置 `running_optimization = true`，
仅在必要时（如确认需要修改共享数据结构时）加锁。

### 方案三：添加 .at() 前的 key 存在性检查

```cpp
for (auto elem : submap_node_has_looped)
{
    if (optimization_problem_->submap_data().count(elem.first) == 0 ||
        optimization_problem_->node_data().count(elem.second) == 0)
    {
        ROS_WARN("Skip loop candidate: submap %u or node %u not in optimizer", 
                  elem.first, elem.second);
        continue;
    }
    // ... 正常的回环验证逻辑 ...
}
```

---

---

## 日志验证结果

通过分析 `feature/learn` 分支的 `output/logs/mapping_slam.log`，发现以下关键证据：

### 统计数据
- 总扫描匹配次数：1796 次
- 成功插入子图次数：667 次
- 被 motion filter 跳过：1129 次
- 优化执行次数：51 次
- 最终结果：530 节点，27 子图，147580 点

### 关键异常：optimization_problem 状态震荡

```
时间(s)   节点数  子图数  约束数   发布点数    说明
271       1      1      1       152        正常（刚开始）
345       92     5      164     25,747     正常（数据增长）
422       1      1      1       152        ← 异常！回退到初始状态！
501       99     5      178     27,431     正常（数据增长）
690       530    27     1,040   147,580    正常（全部数据）
```

**第3次优化时，optimization_problem 中的数据从 92 节点回退到 1 节点！**
这导致发布的地图从 25747 点跌到 152 点 — 在 RViz 中看起来就是"子图消失了"。

### RunOptimization 耗时严重

Timer 设为 5 秒触发一次，但实际触发间隔为 74~189 秒：
- 原因：每次 FastCSM 验证 6 个回环候选，每个约 3 秒，总计约 18 秒
- 建图初期回环候选更多时，阻塞更久

### 重复添加相同约束

bag 播放完后（节点稳定在 530），`submap_node_has_looped` 从不清除，
导致每次优化都重新验证同一组 6 个候选：
- 约束数持续增长：1040 → 1041 → 1042 → ... → 1087
- 同一个回环 (submap_id=0, node_id=510, score=0.870) 被反复接受

### 结论

`feature/learn` 的实际问题是 **optimization_problem 内部数据管理异常** + 
**FastCSM 计算过慢导致 Timer 回调阻塞**，综合表现为：
1. 地图发布频率极低（74~189 秒才更新一次）
2. 地图在"完整"和"几乎空白"之间震荡
3. 最终结果其实是完整的（530节点/147580点），但实时体验很差

`feature/learning_dell` 没有这些问题是因为：
- 使用更快的 RealTimeCorrelativeScanMatcher
- 没有复杂的日志系统开销
- 标志管理更简单

---

## 分析日期

2026-06-08
