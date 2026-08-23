# rmu_gazebo_simulator — 项目交接文档

## 项目概述

基于 **Ignition Gazebo Fortress (gz-sim 6)** 的 RoboMaster 大学赛仿真环境。
本仓库是 SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator 的 **fork**，用于为
**XSmall_point_lio** 算法验证添加 **RoboSense Airy 96 线 LiDAR** 仿真能力。

- 上游仓库：`https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator`
- 本 fork：`https://github.com/Savage-Tea/rmu_gazebo_simulator`
- 主分支：`main`（与上游同步）
- 工作分支：`dev/airy-lidar-simulation`（Airy LiDAR 仿真功能）

## 技术栈

| 组件 | 版本 | 用途 |
|------|------|------|
| ROS 2 | Humble | 消息、launch、节点 |
| Ignition Gazebo | Fortress (gz-sim 6) | 仿真引擎 |
| 渲染引擎 | ogre2 | GPU LiDAR 光线追踪 |
| 构建 | colcon + ament_cmake | ROS 2 包 |
| 部署 | Docker (ros:humble-ros-base) | 跨机器构建 |

## 外部依赖仓库（在 ../ 同级目录）

```
Nav/
├── rmu_gazebo_simulator/   ← 本仓库
├── XSmall_point_lio/       ← 目标 LIO 算法（不改动，消费 RslidarPacket）
├── rs_driver/              ← RoboSense 驱动（解码 RslidarPacket）
├── rslidar_msg/            ← RslidarPacket 消息定义
├── pb2025_sentry_nav/      ← 原导航仓库（旧版点云转换工具参考）
└── pb2025_robot_description/ ← 机器人 SDF xmacro 模板（vcs import 拉取）
```

## 核心功能：Airy 96 线 LiDAR 仿真管线

### 数据流

```
Gazebo GpuLidar 96线 (注入到 front_mid360 link, 10Hz)
  │  spawn_robots.launch.py 运行时用 XML 注入 front_airy_sensor
  ▼
ros_gz_bridge → /<ns>/airy/lidar (sensor_msgs/PointCloud2)
  ▼
airy_packet_bridge (444μs timer 逐4列提取)
  ▼
/lidar/packets (rslidar_msg::RslidarPacket, ~2250Hz)
  ▼
XSmall_point_lio (RoboSensePacketAdapter → rs_driver → LIO)

辅助通道:
  Gazebo IMU sensor → /<ns>/livox/imu → bridge 透传 → /lidar/imu
  GT odometry → /<ns>/chassis_odometry_gt → bridge 透传 → /gt_odometry
```

### Airy 96 线关键规格

- 96 线，360° 水平 FOV，90° 垂直 FOV（-45°~+45°）
- 水平角分辨率 0.4°（Side 模式），垂直角分辨率 0.94°
- 包率 ~2250 Hz（444.44μs/包，Side 模式）
- MSOP 包 1248 字节：8 blocks × 48 channels，96 线用 TwoInOneBlockIterator
- 距离分辨率 0.005m，最大 60m

## 仓库结构

```
rmu_gazebo_simulator/
├── rmu_gazebo_simulator/        # 主包（仿真集成层，已有代码）
│   ├── launch/
│   │   ├── bringup_sim.launch.py    # 总入口
│   │   ├── gazebo.launch.py         # Gazebo 启动
│   │   └── spawn_robots.launch.py   # 机器人生成 + Airy sensor 注入
│   ├── config/
│   │   ├── gz_world.yaml            # 世界选择 + 机器人初始位姿
│   │   ├── ros_gz_bridge.yaml       # ROS↔Gazebo 桥接（含 airy/lidar）
│   │   └── base_params.yaml         # 底盘/云台 PID
│   ├── resource/
│   │   ├── worlds/                  # 世界 SDF（4 个比赛场地 + empty）
│   │   └── models/                  # 场地模型（rmul/rmuc 2024/2025）
│   └── src/airy_link_follower.cpp   # 可选：standalone model 位姿跟随
├── airy_packet_bridge/          # 新包：PointCloud2 → RslidarPacket 转换
├── docs/
│   ├── interfaces.md            # 完整接口规范（6 层架构）
│   └── add-venue-model.md       # 添加新场地模型步骤
└── dependencies.repos           # 外部依赖（vcs import）
```

## 关键实现决策

1. **GpuLidar 用 SDF 注入，不用自定义渲染插件** — Fortress 的原生 `gpu_lidar`
   sensor 类型由 Gazebo Sensors 系统处理，`spawn_robots.launch.py` 用
   `xml.etree.ElementTree` 把 `<sensor type="gpu_lidar">` 注入 `front_mid360` link。

2. **PointCloud2 而非 Gazebo Transport** — `airy_packet_bridge` 订阅 ROS 2 的
   `PointCloud2`（经 ros_gz_bridge 转换），用 444μs wall timer 模拟 Airy 包时序，
   逐 4 列提取并组装 MSOP 包。

3. **IMU 直接透传** — Gazebo IMU sensor 自带 orientation + angular_velocity +
   linear_acceleration（含重力），bridge 不做任何合成，直接转发到 /lidar/imu。

4. **GT odometry 独立输出** — 转发到 /gt_odometry，供 evo_ape 对比 LIO 精度。

## 当前待办事项

### 进行中
- [ ] **添加 RMUC 2026 场地模型** — STP 在 `~/Downloads/UTF-8__RMUC2026_V2.0.0.stp`（1.25 GB），
      步骤见 `docs/add-venue-model.md`
  - [x] 骨架文件已建：`models/rmuc_2026/`（config/sdf/meshes 占位）、`rmuc_2026_world.sdf`、
        `gz_world.yaml` 中 `rmuc_2026` 段（骨架 world 不含 standalone airy_lidar，走注入方案）
  - [x] STP→STL 转换（pythonocc 曲面细分 → 15mm 体素聚类 → MeshLab 二次误差减面，
        25 万面 / 12MB；原始 STP 单位毫米、地面在 z=-1941mm，已平移归零；
        mesh 包围盒 29.75×16×3.8m，含地面大面 + 围墙 + 场地部件）
  - [x] `model.sdf` link pose 已按 rmuc_2025 约定（网格角点对齐原点）设 14.88 6.38 0；
        机器人初始位姿暂设场地中心 (14.88, 8.0, 0.28)
  - [ ] 启动验证（本机无 ROS/ign，需在 Docker 或装有环境机器上跑）；确认场地布局无异常后
        再把 `world:` 切到 `rmuc_2026`

### 待优化（按优先级）
- [ ] **P0** 删除废弃的 `airy_gazebo_plugin/` 包（被 Fortress 架构替代）
- [ ] **P1** 清冗余：standalone `airy_lidar` model + `airy_link_follower` 与注入方案二选一
- [ ] **P2** 多机器人 topic 冲突：`/lidar/packets`、`/lidar/imu` 改为 `/<ns>/` 前缀
- [ ] **P3** 噪音模型增强（range-dependent noise + drop pattern）

## 构建与测试

```bash
# 构建（需要 Ignition Fortress 环境）
colcon build --symlink-install \
    --packages-select rmu_gazebo_simulator airy_packet_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 启动仿真
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 启动 bridge
ros2 launch airy_packet_bridge airy_packet_bridge.launch.py

# 验证
ros2 topic hz /lidar/packets    # 期望 ~2250Hz
ros2 topic hz /lidar/imu        # 期望 ~1000Hz

# 启动 LIO（../XSmall_point_lio）
ros2 launch xspl xspl.launch.py lidar_type_str:=RSAIRY

# 精度验证
ros2 bag record /Odometry /gt_odometry
evo_ape bag output_bag/ /gt_odometry /Odometry -va
```

## 关键坑与教训

1. **Entity 时序**：机器人通过 `ros_gz_sim create` 动态生成，晚于 world plugin 的
   `Configure`。任何需要引用机器人 link 的 plugin 必须用 PostUpdate 延迟查找。

2. **transport::Node 生命周期**：`ignition::transport::Node().Advertise()` 临时对象
   析构会导致 publisher 失效，必须用 `shared_ptr<Node>` 持久持有。

3. **0.5ms 物理步长**：Airy 444μs 包率要求 `max_step_size=0.0005`，所有 world SDF
   必须保留此配置。

4. **ogre2 渲染**：GPU LiDAR 需要 ogre2，2024 世界已从 ogre 升级，勿改回。

5. **MSOP 时间戳格式**：Airy 用分离的 uint32 usec + uint48 sec 字段，不是组合的
   usec-since-epoch。

## 外部文档

- 设计文档：`docs/superpowers/specs/2026-07-02-airy-lidar-simulation-design.md`
- 接口规范：`docs/interfaces.md`
- Airy datasheet：`~/Downloads/robosense.pdf`
- rs_driver Airy 解码器：`../rs_driver/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp`
