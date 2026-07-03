# Airy LiDAR Simulation for XSmall Point-LIO — Design Doc

## 1. 目标

在 rmu_gazebo_simulator 中模拟 **RoboSense Airy 96 线 LiDAR** 的原始 MSOP 数据包流，
供 **XSmall Point-LIO** 实时消费，用于验证 LIO 算法的里程计精度。

Gazebo 提供 ground truth 里程计 (`chassis_odometry_gt`)，可与 XSmall Point-LIO
输出的 `/Odometry` 做定量对比。

## 2. 约束

- **rmu_gazebo_simulator 改动最小化**：仅 world SDF（2 行/世界），不对已有包代码做任何修改
- **不改动 XSmall_point_lio**：直接消费 RslidarPacket
- **利用 Ignition Gazebo 渲染引擎**：GPU 射线投射（ogre2），不做纯 CPU 光线追踪
- **包率正确**：~2250 Hz（Airy Side 模式 444.44 μs/包）
- **扫描模式正确**：96 线、360°×90° FOV、0.4° 水平角分辨率
- **时序正确**：每个点的时间戳按 Airy block firing time 计算

## 3. 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│ rmu_gazebo_simulator repo (Savage-Tea fork)                      │
│                                                                  │
│ ┌────────────────────────────────────────────────────────────┐   │
│ │ Gazebo Simulation (step = 0.5 ms)                           │   │
│ │                                                            │   │
│ │  airy_gazebo_plugin (System Plugin, 新包)                   │   │
│ │  ┌──────────────────────────────┐                          │   │
│ │  │ Per step (~2000Hz):          │                          │   │
│ │  │  读取 LiDAR link world pose  │                          │   │
│ │  │  旋转 azimuth (600RPM)       │                          │   │
│ │  │  发射 96 垂直射线 @ 1 水平角  │                          │   │
│ │  │  GPU 渲染 → 96 个 range      │                          │   │
│ │  │  球→笛卡尔 → Publish (GT)    │                          │   │
│ │  └──────────────┬───────────────┘                          │   │
│ │                 │ ignition::msgs::PointCloudPacked          │   │
│ │                 │ (每 block: 96 点, ~2250Hz)                │   │
│ └─────────────────┼──────────────────────────────────────────┘   │
│                   │                                              │
│ ┌─────────────────▼──────────────────────────────────────────┐   │
│ │ airy_packet_bridge (ROS 2 Node, 新包)                       │   │
│ │ ┌──────────────────────────────┐                           │   │
│ │ │ 订阅 Gazebo Transport PCP    │                           │   │
│ │ │ 累积 4 个 azimuth → 1 MSOP 包│                           │   │
│ │ │ 编码 Airy MSOP 字节 (1248B)  │                           │   │
│ │ │ 发布 RslidarPacket (~2250Hz) │                           │   │
│ │ └──────────────────────────────┘                           │   │
│ │                                                            │   │
│ │ IMU 透传: /livox/imu → /lidar/imu                          │   │
│ └─────────────────┬──────────────────────────────────────────┘   │
│                   │ /lidar/packets (rslidar_msg::RslidarPacket)  │
│                   │ /lidar/imu    (sensor_msgs::Imu)             │
└───────────────────┼──────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────┐
│ XSmall_point_lio (独立 repo, 不改动)                              │
│                                                                  │
│ /lidar/packets → RoboSensePacketAdapter → rs_driver decode       │
│               → Point {xyz, intensity, ring, timestamp}          │
│ /lidar/imu    → ImuMsg {acc, gyro, timestamp}                    │
│                    │                                             │
│                    ▼                                             │
│               Lio::handle_once()                                  │
│               → /Odometry (nav_msgs/Odometry)                    │
│               → /cloud_registered (sensor_msgs/PointCloud2)      │
└──────────────────────────────────────────────────────────────────┘
```

## 4. Airy 96 线规格（Side 模式，仿真目标）

| 参数 | 值 | 来源 |
|------|-----|------|
| 线数 | 96 | rs_driver decoder_RSAIRY |
| 垂直 FOV | 90° (-45° ~ +45°) | Airy datasheet |
| 垂直角分辨率 | 0.9375° (90°/96) | 推导 |
| 水平 FOV | 360° | Airy datasheet |
| 水平角分辨率 | 0.4° | Side 模式 (rs_driver install_mode=0x1) |
| 旋转速率 | 600 RPM (10 Hz) | Airy datasheet |
| Block 时长 | 111.080 μs | rs_driver decoder_RSAIRY |
| 包时长 | 444.320 μs (4 blocks) | rs_driver |
| 包率 | ~2250 Hz | 1 / 444.32μs |
| 每包 azimuth 组 | 4 | 8 blocks / 2 (TwoInOneBlockIterator) |
| 每圈包数 | 225 | 100ms / 444.44μs |
| MSOP 包大小 | 1248 字节 | rs_driver decoder_RSAIRY |
| 距离分辨率 | 0.005 m | rs_driver DISTANCE_RES |
| 最大距离 | 60 m | rs_driver |
| Firing 组 | 每 8 通道同时发射 (12 组) | rs_driver firing_tss_side |
| Channel 布局 | 奇数 block: ring 48-95, 偶数 block: ring 0-47 | rs_driver TwoInOneBlockIterator |

### Timestamp 计算

```
point.timestamp = pkt_ts + (blk % 4) * BLOCK_DURATION + CHAN_TSS[ring]

其中:
  BLOCK_DURATION = 111.080e-6 s
  CHAN_TSS[12] = {0.0, 11.424, 22.848, 34.272, 45.696, 57.120,
                   68.544, 79.968, 91.392, 99.008, 110.432, 119.856} μs  (Side 模式)
  每 8 通道一组 (12 组 = 96 线)，组内 firing time 相同
  CHAN_TSS 索引: ring / 8 (整数除法)
```

## 5. 组件设计

### 5.1 `airy_gazebo_plugin` — Gazebo System Plugin

**文件结构：**

```
airy_gazebo_plugin/
├── CMakeLists.txt
├── package.xml
├── include/airy_gazebo_plugin/
│   └── airy_gpu_lidar.hpp
└── src/
    └── airy_gpu_lidar.cpp
```

**核心职责：** 每个 simulation step 向 Gazebo 渲染引擎提交一组 96 条射线，
获取 range 值，以 PointCloudPacked 发布到 Gazebo Transport。

**关键设计决策：**

1. **单水平角、96 垂直角**：每个 block 只发射 1 个水平方向的 96 条射线。
   GPU 在 <100μs 内完成，不会阻塞仿真循环。

2. **亚步长精度**：step 可能跨越多个 block 边界（高动态或 step=1ms 时），
   用 while 循环一次性处理所有待发射的 block。每个 block 的时间戳用
   `CHAN_TSS` 精确插值。

3. **Gazebo Transport 直出**：不经过 ros_gz_bridge。airy_packet_bridge
   通过 ignition::transport 直接订阅，零拷贝。

**伪代码：**

```cpp
void PostUpdate(const UpdateInfo &info, EntityComponentManager &ecm) {
    // 1. 读取 LiDAR world pose (从 ECS)
    auto *pose = ecm.Component<WorldPose>(lidar_link_entity_);

    // 2. 旋转 azimuth
    azimuth_accumulator_ += 3600.0 * info.dt;  // 600RPM = 3600°/s

    // 3. 循环: 发射所有跨过的 block
    while (azimuth_accumulator_ >= AZIMUTH_PER_BLOCK) {
        azimuth_accumulator_ -= AZIMUTH_PER_BLOCK;
        current_azimuth_ += AZIMUTH_PER_BLOCK;  // ~0.4°

        // 3a. 射线配置
        gpu_rays_->SetWorldPose(*pose);
        gpu_rays_->SetHorizontalAngle(current_azimuth_ * M_PI/180);

        // 3b. GPU 渲染 (异步, <100μs for 96 rays)
        gpu_rays_->Update();

        // 3c. 获取结果 (96 个 range)
        auto ranges = gpu_rays_->Ranges();

        // 3d. 球→笛卡尔
        PointCloudPacked msg;
        // ... fill msg with 96 xyz points ...

        // 3e. 发布到 Gazebo Transport
        pub_.Publish(msg);
    }
}
```

**SDF 参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_name` | `red_standard_robot1` | 机器人 Gazebo model name |
| `lidar_link` | `front_airy` | LiDAR 所在的 link name |
| `rpm` | `600` | 旋转速率 |
| `max_range` | `60.0` | 最大测距 (m) |
| `min_range` | `0.1` | 最小测距 (m)，盲区 |

**Gazebo Transport topic：**

```
/world/default/model/{robot_name}/link/{lidar_link}/sensor/airy_lidar/scan/points
```

**依赖：**

- `ignition-gazebo6` — System plugin 接口、ECS
- `ignition-rendering6` — GpuRays API
- `ignition-transport11` — Gazebo Transport 发布

### 5.2 `airy_packet_bridge` — Bridge 节点

**文件结构：**

```
airy_packet_bridge/
├── CMakeLists.txt
├── package.xml
├── include/airy_packet_bridge/
│   └── airy_packet_bridge.hpp
├── src/
│   └── airy_packet_bridge.cpp
├── config/
│   └── airy_packet_bridge.yaml
└── launch/
    └── airy_packet_bridge.launch.py
```

**核心职责：** 订阅 Gazebo Transport 的 block 级 PointCloudPacked，
累积 4 个 azimuth sample 后组装为一个完整的 Airy MSOP 字节包，
以 `rslidar_msg::RslidarPacket` 发布。同时透传 IMU 数据。

**MSOP 包结构（1248 字节）：**

```
Offset | Size  | Field
───────┼───────┼────────────────────────────────────────
0      | 4     | Magic: 0x55 0xAA 0x05 0x5A
4      | 12    | Reserved + pkt_cnt
16     | 2     | data_type: [0]=0 (point cloud)
18     | 10    | UTC timestamp (6B sec + 4B usec)
28     | 1     | lidar_type
29     | 1     | lidar_mode: 0x02 (96-line)
30     | 12    | Reserved + temperature
─────────────── Block 0 (148B) ──────────────────────
42     | 2     | Block magic: 0xFF 0xEE
44     | 2     | azimuth (uint16, 0.01° 单位)
46     | 144   | 48 channels × 3B (2B distance + 1B intensity)
────────────────  Blocks 1-7  ────────────────────────
190-  | 1036  | ... (7 more blocks)
───────┼───────┼────────────────────────────────────────
1226   | 6     | tail
1232   | 16    | reserved
───────┴───────┴────────────────────────────────────────
TOTAL: 1248 bytes
```

**Channel 编码（3 字节）：**

```
Byte 0: distance[7:0]
Byte 1: distance[13:8] | feature[1:0]<<6    (feature = 0 for normal point)
Byte 2: intensity (uint8, 0-255)
```

**96 线 Block 映射：**

```
Block 偶/奇 → ring 范围
─────────────────────────
blk 0 (偶)  → rings 0-47
blk 1 (奇)  → rings 48-95
blk 2 (偶)  → rings 0-47
blk 3 (奇)  → rings 48-95
...
```

每 2 个 block 共享同一个 azimuth（来自同一个 AzimuthSample）。

**环形缓冲逻辑：**

```cpp
void onGpuLidarBlock(const PointCloudPacked &msg) {
    // 1. 解析 → AzimuthSample
    AzimuthSample sample = parseFromPacked(msg);

    // 2. 写入环形缓冲 (4 槽)
    azimuth_ring_[write_idx_++ % 4] = sample;

    // 3. 累积 4 个 → 发布 1 个 MSOP 包
    if (write_idx_ % 4 == 0) {
        RslidarPacket pkt;
        buildMsopPacket(azimuth_ring_[0..3], pkt);
        packet_pub_->publish(pkt);
    }
}
```

**IMU 处理：** 从 ros_gz_bridge 接收 `sensor_msgs/Imu`，直接重发布到 `/lidar/imu`。
不做坐标变换或时间戳处理（XSmall_point_lio 内使用 `extrinsic_T/R` 参数处理外参）。

**参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_name` | `red_standard_robot1` | Gazebo 模型名 |
| `imu_topic` | `livox/imu` | 输入的 IMU ROS topic |
| `use_sim_time` | `true` | 使用 Gazebo 仿真时间 |

**发布 topic：**

| Topic | 类型 | 频率 |
|-------|------|------|
| `/lidar/packets` | `rslidar_msg::RslidarPacket` | ~2250 Hz |
| `/lidar/imu` | `sensor_msgs::Imu` | ~1000 Hz |

**依赖：**

- `rclcpp` / `rclcpp_components` — ROS 2 节点
- `rslidar_msg` — RslidarPacket 消息定义
- `sensor_msgs` — IMU 消息
- `ignition-transport11` — Gazebo Transport 订阅

## 6. 构建系统

### 6.1 CMakeLists.txt

`airy_gazebo_plugin` 链接 `ignition-gazebo6`、`ignition-rendering6`、`ignition-transport11`。
安装 target 到 `lib/`（Gazebo plugin 搜索路径需在 `GAZEBO_PLUGIN_PATH` 中）。

`airy_packet_bridge` 链接 `ignition-transport11`、`rclcpp`、`rslidar_msg`。
注册为 `rclcpp_components` 节点。

### 6.2 环境变量

```bash
# 加入 ~/.zshrc 或 setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros_ws/install/airy_gazebo_plugin/lib
```

由 `rmu_gazebo_simulator/env-hooks/gazebo.dsv.in` 扩展。

## 7. 仓库改动清单

| 文件 | 操作 | 说明 |
|------|:---:|------|
| `airy_gazebo_plugin/` | **新增** | Gazebo System Plugin 包 |
| `airy_packet_bridge/` | **新增** | Bridge 节点包 |
| `resource/worlds/*_world.sdf` (4 文件) | **修改** | 每文件 +`max_step_size` + `<plugin>` |
| `env-hooks/gazebo.dsv.in` | **修改** | 追加新 plugin 路径到 `GAZEBO_PLUGIN_PATH` |
| `dependencies.repos` | **修改** | 追加 `rslidar_msg` 仓库 |
| `Dockerfile` | **修改** | 追加新包构建 + `ignition-rendering-dev` |
| `.gitignore` | **修改** | 追加 `docs/superpowers/` |
| `rmu_gazebo_simulator/` 已有包 | **不改** | 零代码变更 |

## 8. rmu_gazebo_simulator World SDF 改动

每个世界文件新增 2 处：

```xml
<world name='default'>
    <!-- 改动 1: 0.5ms 步长 -->
    <physics type="ode">
        <max_step_size>0.0005</max_step_size>
        <real_time_factor>1.0</real_time_factor>
        <real_time_update_rate>2000</real_time_update_rate>
    </physics>

    <!-- 已有插件不变 -->
    <plugin filename="libignition-gazebo-physics-system.so" .../>
    <plugin filename="libignition-gazebo-sensors-system.so" .../>
    <!-- ... -->

    <!-- 改动 2: Airy GPU LiDAR -->
    <plugin filename="libairy_gazebo_plugin.so"
            name="airy::AiryGpuLidarPlugin">
        <robot_name>red_standard_robot1</robot_name>
        <lidar_link>front_airy</lidar_link>
        <rpm>600</rpm>
        <max_range>60.0</max_range>
        <min_range>0.1</min_range>
    </plugin>

    <!-- 场地模型不变 -->
    <model name='rmuc_2025'>...</model>
</world>
```

## 9. Fork 与远程配置

```
origin   → git@github.com:Savage-Tea/rmu_gazebo_simulator.git (SSH, 推送目标)
upstream → https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator.git (HTTPS, 只读)
```

`git push` 默认推送到 Savage-Tea fork，不会触动上游仓库。

## 10. XSmall Point-LIO 集成

### 10.1 下载

```bash
# 已有 (本地):
#   Nav/XSmall_point_lio/
#   Nav/rs_driver/
#   Nav/rslidar_msg/

# 如需要 clone:
cd ~/Documents/RoboMaster/S26-27/Nav
git clone git@github.com:Savage-Tea/XSmall_point_lio.git
```

### 10.2 依赖安装

```bash
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libeigen3-dev \
    libomp-dev
```

### 10.3 构建

```bash
cd ~/ros_ws
colcon build --symlink-install \
    --packages-select rslidar_msg rs_driver xspl \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.zsh
```

### 10.4 仿真启动

```bash
# Terminal 1: Gazebo
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# Terminal 2: Bridge + LIO
ros2 launch airy_packet_bridge airy_packet_bridge.launch.py \
    namespace:=red_standard_robot1
ros2 launch xspl xspl.launch.py \
    lidar_type_str:=RSAIRY

# Terminal 3: Ground truth 录制 (验证用)
ros2 bag record \
    /Odometry \
    /red_standard_robot1/chassis_odometry_gt
```

### 10.5 精度验证

```bash
# 用 evo 对比轨迹
evo_traj bag output_bag/ \
    /Odometry \
    /red_standard_robot1/chassis_odometry_gt \
    --ref /red_standard_robot1/chassis_odometry_gt \
    -p --plot_mode=xyz

# evo_ape: 绝对位姿误差
evo_ape bag output_bag/ \
    /red_standard_robot1/chassis_odometry_gt \
    /Odometry -va --plot
```

## 11. 已知限制 & 后续工作

| 限制 | 影响 | 缓解措施 |
|------|------|---------|
| GPU LiDAR 无反射率 | intensity 恒为 0 | XSmall_point_lio 的 reflectivity filter 需要关闭 |
| 0.5ms step 可能影响大型场景物理 | 多机器人 + 大场地 | 仅用单机器人测试，后续可调 `real_time_factor` |
| Ray 从 LiDAR link 中心发射，无 lens center 偏移 | 近距离精度 ~1cm 误差 | 对 LIO 验证影响可忽略 |
| `pb2025_robot_description` 中需添加 `front_airy` link | 需改动外部包 | 备选：复用已有 `front_mid360` link，通过 SDF `<lidar_link>` 参数指定；如当前模型无此 link，plugin 启动时输出 WARN 并跳过，不影响 Gazebo 运行 |
| Gazebo Transport 消息大小 | 96 点 × 12B/点 = 1.2KB, ~2250Hz = 2.6MB/s | 共享内存，无性能瓶颈 |

## 12. 实施顺序

1. 创建 `airy_gazebo_plugin` 包骨架 (CMakeLists, package.xml, 头文件, 源文件)
2. 创建 `airy_packet_bridge` 包骨架
3. 修改 world SDF (4 文件 × 2 行)
4. 修改 `env-hooks/gazebo.dsv.in`、`dependencies.repos`、`Dockerfile`
5. `colcon build` 验证编译
6. 启动仿真，验证 `/lidar/packets` 有输出
7. 用 `rs_driver` demo 验证 packet 可解码
8. 启动 XSmall_point_lio，录制 bag，对比 ground truth
