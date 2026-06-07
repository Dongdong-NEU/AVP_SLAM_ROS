# AVP_SLAM_ROS
    停车场环境中的语义建图与定位算法，代码实现参考cartographer。
    [video](https://www.bilibili.com/video/BV1aG4y1i7EC?share_source=copy_web&vd_source=1cd3f6f1bab3232928b0a4cc0ae193ad)

## 1.Prerequisites
    1.1 开发于ubuntu18.04 + ros melodic
    1.2 依赖库 ceres-solver

## 2.Build on ROS
    cd ~/catkin_ws/src
    git clone https://github.com/guyupan1911/AVP_SLAM_ROS.git
    cd ../
    catkin_make
    source devel/setup.bash

## 3.Build on docker environment
    1. pull image
    sudo docker pull fdko11/ros:bionic-melodic-cartographer
    2. create container
    ./docker/dev_start.sh (run once)
    3. start container
    docker start AVP
    4. exec container
    ./docker/dev_into.sh
    
## 4.run semantic mapping

### 4.1 编译

```bash
cd ~/catkin_ws

# 只编译 avp 包（推荐，避免其他包干扰）
catkin_make -DCATKIN_WHITELIST_PACKAGES="avp" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 或全量编译
# catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 4.2 运行

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch avp mapping.launch
```

> **zsh 用户注意**：若使用 zsh 报错 `setup.bash: 没有那个文件或目录`，可用：
> ```bash
> bash -c 'source devel/setup.bash && roslaunch avp mapping.launch'
> ```

launch 会同时启动三个节点：

| 节点 | 作用 |
|------|------|
| `mapping_node` | SLAM 建图主节点 |
| `rosbag play` | 回放 `data/path.bag`（里程计 + 语义点云） |
| `rviz` | 可视化（加载 `launch/mapping.rviz` 配置） |

### 4.3 输入话题（由 bag 提供）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odometry_noised` | `geometry_msgs/PoseStamped` | 带噪声的里程计 (x, y, yaw) |
| `/scan_semantic_points` | `sensor_msgs/PointCloud2` | 语义点云（车位线、箭头等） |

### 4.4 输出话题（RViz 可视化）

| 话题 | 说明 |
|------|------|
| `/semantic_map` | 全局语义地图点云（绿色） |
| `/path_estimated` | 扫描匹配后的轨迹 |
| `/path_estimated_after_spa` | 全局优化后的轨迹 |
| `/path_noise` | 原始噪声里程计轨迹 |
| `/accumulated_semantic_scan` | 当前帧累积的语义扫描 |

### 4.5 运行过程

1. bag 播放数据 → `mapping_node` 实时建图
2. 每 5 秒触发一次全局优化（回环检测 + SPA）
3. bag 播完后节点仍在运行（定时优化继续）
4. **按 Ctrl+C 正常退出**，析构函数会保存最终地图

### 4.6 查看结果

```bash
# 查看日志
cat output/logs/mapping_slam.log | grep "LOOP\|SPA\|OPTIMIZE"

# 用 pcl_viewer 查看保存的地图
pcl_viewer output/semantic_map_final.pcd
```

## 5.日志与地图保存

建图节点会将 SLAM 过程写入文件，并在优化后/退出时保存点云地图，便于离线分析回环与 SPA 效果。

### 5.1 输出路径（默认）

| 类型 | 路径 |
|------|------|
| 运行日志 | `$(find avp)/output/logs/mapping_slam.log` |
| 最新地图（每次优化覆盖） | `$(find avp)/output/semantic_map_latest.pcd` |
| 最终地图（节点退出时保存） | `$(find avp)/output/semantic_map_final.pcd` |

### 5.2 可调 ROS 参数（`mapping` 节点私有命名空间）

在 `launch/mapping.launch` 中配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `log_dir` | `$(find avp)/output/logs` | 日志目录 |
| `log_file` | `mapping_slam.log` | 日志文件名 |
| `enable_file_log` | `true` | 是否写文件（仍会输出到终端） |
| `map_save_path` | `$(find avp)/output/semantic_map_final.pcd` | 退出时保存的最终地图 |
| `save_map_each_optimization` | `true` | 每次 SPA 后保存 `_latest.pcd` |
| `save_map_on_shutdown` | `true` | 退出时保存最终地图 |

### 5.3 日志分类

| 分类 | 含义 |
|------|------|
| `SCAN_MATCH` | 前端 Ceres 扫描匹配成功/失败 |
| `INSERT` | 子图插入或运动过滤跳过 |
| `NODE` | 新轨迹节点加入位姿图 |
| `LOOP` | 回环候选、接受/拒绝及匹配得分 |
| `SPA` | Ceres 全局优化迭代与代价 |
| `OPTIMIZE` | 定时优化触发与统计 |
| `MAP_PUB` / `MAP_SAVE` | 地图发布与 PCD 保存 |
