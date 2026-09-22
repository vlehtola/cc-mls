# CC MLS SLAM 单包源码

本包融合FAST-LIO2双遍估计、动态去除和静态点云回环后端。一个package.xml、一个CMakeLists.txt，内部构建cc_mls_frontend、cc_mls_backend和cc_mls_dynamic，无需分别安装三个SLAM项目。ROS、GTSAM及设备驱动仍是外部依赖。

## 安装和验证

目标：Ubuntu20.04、ROS1 Noetic、GCC9/C++17、GTSAM4.2（系统Eigen）。先配置好ROS环境，将cc_mls_slam放入工作区src：

```bash
cd ~/catkin_ws
bash src/cc_mls_slam/scripts/install_new_machine.sh "$PWD"
bash src/cc_mls_slam/scripts/verify_installation.sh "$PWD"
source install/setup.bash
```

[新电脑安装说明](docs/新电脑安装.md)列出依赖和官方参考。当前Windows环境没有ROS/Linux工具链；新增代码、安装脚本、C++测试和rosbag尚未实测，不能宣称已验证可运行。

## 三个传感器入口

| 传感器 | 启动 | 输入 |
|---|---|---|
| Ouster128 | roslaunch cc_mls_slam mapping_ouster128.launch | PointCloud2，默认相对纳秒字段t，数值ring；支持UINT8/UINT16 ring和行填充 |
| Livox Horizon | roslaunch cc_mls_slam mapping_horizon.launch | 旧版livox_ros_driver/CustomMsg |
| Livox HAP | roslaunch cc_mls_slam mapping_hap.launch | ROS1版livox_ros_driver2，xfer_format=0发布PointCloud2 |

每个launch加载对应ouster128.yaml/horizon.yaml/hap.yaml，再加载fusion.yaml。HAP现在直接读取driver2的PointCloud2，不接收driver2/CustomMsg；需要安装和启动对应设备驱动，但不需要CustomMsg转换节点。官方driver2 PointCloud2中timestamp为相对纳秒，代码默认乘1e-6转为毫秒。接入参考见[运行问题修复记录](docs/运行问题修复记录.md)。旧版Livox驱动仍是单包构建依赖。

三个入口均保留rviz、backend、covisibility、sensor_config、method_config和save_directory参数。Ouster/HAP额外提供max_scan_points（默认600000）。传感器外参为待标定占位值；话题、IMU、视场、量程、盲区和噪声必须核对。HAP部分调参仍继承Horizon起始配置，不能当作实测设备参数。

```bash
rosparam set use_sim_time true
roslaunch cc_mls_slam mapping_ouster128.launch rviz:=false \
  save_directory:=/absolute/path/to/results
# 另一终端
rosbag play /absolute/path/to/data.bag --clock
```

时间回退和队列容量耗尽会终止当前运行，需重新启动SLAM。原始IMU时间同步必须正确；不能用循环bag继续沿用旧地图。

## 流程和输出

IMU传播/去畸变一次 → 全点第一遍LIO → 动态检测 → 恢复同一预测状态与协方差 → 静态点第二遍LIO → 更新静态地图及历史深度 → 静态关键帧回环/共视优化。

动态分类为ENTERING、LEAVING、APPROACHING、RECEDING。历史窗口可用且第二遍成功后，默认替换临时启动地图；替换前不提交后端静态关键帧、不累计PCD。该处理不能保证动态误检为零，也不回溯重算启动位姿。

主要话题：/Odometry、/fusion/static_lidar、/fusion/dynamic_lidar、/fusion/lidar_odometry、/fusion/stats、/aft_pgo_path、/aft_pgo_odom、/aft_pgo_map。动态点云为合并输出，未增加四个分类话题。前端日志在运行时创建的ROS用户目录，可通过output_directory设置；PCD按chunk_N.pcd分块，剩余点保存为scans.pcd。后端save_directory须显式指定才保存轨迹和关键帧。

## 资源限制和复现范围

内部队列、前端轨迹、PCD块、动态历史点数组、后端关键帧数量和点总数均有容量保护。后端达到容量会终止当前运行，不是无限时长的图优化或自动分段地图。限制及动作见[运行问题修复记录](docs/运行问题修复记录.md)。

论文的聚类窄带采样、噪声模型和PGO反馈仍存在复现差异；不要把流程接入或变量重命名等同于完整论文实验复现。

- [运行问题修复记录（当前行为）](docs/运行问题修复记录.md)
- [全库审查快照](docs/全库环境与运行审查.md)
- [算法与复现差异](docs/算法与复现差异.md)
- [论文后端符号对应](docs/论文后端符号对应.md)
- [四种动态运动类别](docs/四种动态运动类别.md)

动态聚类模块的文件职责、执行顺序和验证边界见 [动态聚类代码结构](docs/动态聚类代码结构.md)。

论文相关内部命名已统一，详见 [论文术语与代码命名](docs/论文术语与代码命名.md)。外部 ROS 配置键保持兼容。
