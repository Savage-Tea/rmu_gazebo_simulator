# RMU Gazebo Simulator — 团队讲解提纲

> 目标听众: 小组研发成员 | 时长: 约 45-60 分钟 | 日期: 2026-07-06

## 第一部分: 这是什么 (5 分钟)

### 1.1 一句话定位
基于 ROS 2 Humble + Ignition Gazebo Fortress 的 RoboMaster University 竞赛仿真环境，
用于机器人算法（导航、感知、SLAM）的开发、测试和定量评估。

### 1.2 能做什么
- 在 4 个 RMU 比赛场地上运行仿真
- 搭载完整传感器配置的机器人（Mid360 LiDAR、IMU、RPLidar A2、工业相机、**Airy 96线 LiDAR**）
- Ground truth 里程计 → 算法精度定量对比 (evo)
- 网页端联机对战 + 裁判系统

### 1.3 演示 (建议)
- 运行 `scripts/sim.sh`，展示 Gazebo GUI 和机器人
- 跑一个 `test_chassis_cmd.py` 展示机器人移动

---

## 第二部分: 仓库怎么组织的 (10 分钟)

### 2.1 三包结构

```
rmu_gazebo_simulator/          ← 主元包: launch、world、配置、C++ 插件
airy_gazebo_plugin/            ← Airy GPU LiDAR 渲染插件 (我们新写的)
airy_packet_bridge/            ← Gazebo→ROS MSOP 编码桥接 (我们新写的)
```

**重点**: 区分上游代码 (Lihan Chen 的 `rmu_gazebo_simulator`) 和我们的增量 (两个新包 + airy_link_follower)。

### 2.2 启动流程
```
bringup_sim.launch.py
  ├─ gazebo.launch.py        → 启动 Gazebo Fortress + 世界
  ├─ spawn_robots.launch.py  → 生成机器人 + 传感器 + bridge
  └─ referee_system.launch.py → 裁判系统
```

### 2.3 关键配置文件
| 文件 | 作用 |
|------|------|
| `config/gz_world.yaml` | 选择场地 + 机器人出生点 |
| `config/ros_gz_bridge.yaml` | Gazebo ↔ ROS 2 话题映射 |
| `resource/worlds/*.sdf` | 世界定义 (物理、插件、场地模型) |

---

## 第三部分: 数据怎么流动的 — 核心架构图 (10 分钟)

> **这是最重要的部分。用这张图贯穿整个讲解。**

### 3.1 数据流图

```
                    Gazebo 仿真引擎 (Ogre2 GPU 渲染)
                              │
        ┌─────────────────────┼─────────────────────────┐
        ▼                     ▼                         ▼
   Mid360 LiDAR        Airy GpuLidar              其他传感器
   (原生传感器)        (SDF 注入传感器)          (相机/RPLidar)
        │                     │                         │
        ▼                     ▼                         ▼
   ros_gz_bridge         ros_gz_bridge             ros_gz_bridge
        │                     │                         │
        ▼                     ▼                         ▼
 /<ns>/livox/lidar     /<ns>/airy/lidar          /<ns>/*
 (PointCloud2)         (PointCloud2 900×96)
                              │
                              ▼
                      AiryPacketBridge
                        (ROS 2 Node)
                              │
                    ┌─────────┴────────┐
                    ▼                  ▼
            /lidar/packets        /lidar/imu
         (RslidarPacket)      (sensor_msgs)
                    │                  │
                    └────────┬─────────┘
                             ▼
                    XSmall Point-LIO
                             │
                             ▼
                       /Odometry
                             │
              ┌──────────────┴──────────────┐
              ▼                             ▼
     /<ns>/chassis_odometry_gt       evo 定量对比
         (Ground Truth)
```

### 3.2 关键点
- **两条 LiDAR 通路**: Mid360 (导航) 和 Airy (LIO 建图) 同时运行
- **AiryPacketBridge**: 结构化的 PointCloud2 → 符合 Airy 硬件协议的 MSOP 字节流
- **XSmall Point-LIO 不改一行代码**: 它以为自己在读真实硬件数据
- **GT odometry 来自 Gazebo**: 完全准确的真值，用于评估 LIO 精度

---

## 第四部分: 接口五层模型 (15 分钟)

> **团队其他成员做开发时，只需要找到对应的接口层。**

### 4.1 五层总览

```
Layer 5: ROS 2 应用接口   ← 算法开发者入口
Layer 4: ros_gz_bridge    ← Gazebo↔ROS 协议转换
Layer 3: Gazebo Transport ← 仿真内部消息
Layer 2: ECS 组件接口     ← 插件开发入口
Layer 1: SDF 配置接口     ← 声明式配置
Layer 0: Launch 组装      ← 启动组装
```

### 4.2 Layer 0: 怎么启动、怎么换场地、怎么加机器人
- 修改 `gz_world.yaml` → 换场地
- 取消注释 blue robot → 双机器人
- `ros2 launch rmu_gazebo_simulator bringup_sim.launch.py` → 一键启动

### 4.3 Layer 1: 传感器怎么加 (SDF 注入模式)
- 在 `spawn_robots.launch.py` 中仿照 Airy 注入代码
- `<sensor type="gpu_lidar" name="...">` XML 片段
- 不需要写 C++ 代码的传感器添加方式

### 4.4 Layer 2: 怎么写 Gazebo 插件 (ECS 接口)
- 三个必须实现的接口: `ISystemConfigure` + `ISystemPreUpdate/PostUpdate`
- ECM 读写模式: 读用 `Each<>`, 写用 `const_cast` (PostUpdate 限制)
- **延迟查找模式**: 机器人动态生成 → Configure 时不存在 → PostUpdate 重试
- 展示 `AiryGpuLidarPlugin` 作为参考实现

### 4.5 Layer 3: Gazebo Transport (仿真内部通信)
- Topic 命名: `/world/{world}/model/{model}/link/{link}/sensor/{sensor}/...`
- 消息类型: `PointCloudPacked`, `Odometry`, `IMU`, `Image`, `LaserScan`
- 发布器生命周期: `shared_ptr<Node>` 防止悬空指针

### 4.6 Layer 4: ros_gz_bridge (Gazebo ↔ ROS)
- 配置格式: `ros_topic → gz_topic + 类型 + 方向`
- `<robot_name>` 占位符替换机制
- 当前 8 条桥接线路

### 4.7 Layer 5: 算法开发者能拿到什么
- `/red_standard_robot1/chassis_odometry_gt` — 真值里程计
- `/red_standard_robot1/airy/lidar` — Airy 96线点云
- `/lidar/packets` — Airy MSOP 包 (XSmall Point-LIO 直接消费)
- 命名空间规则: 传感器在机器人 NS 下，LIO 输出在全局

---

## 第五部分: Airy LiDAR 仿真深度解析 (10 分钟)

### 5.1 为什么需要这么复杂？

```
真实 Airy LiDAR 数据流:
  硬件 → MSOP 包 (1248B@2250Hz) → rs_driver → PointCloud → LIO

仿真必须复现:
  仿真 → ??? → MSOP 包 (1248B@2250Hz) → rs_driver → PointCloud → LIO
```

### 5.2 三层组件协作

| 组件 | 职责 | 输出 |
|------|------|------|
| `gpu_lidar` 传感器 (SDF) | 10Hz 渲染 900×96 点云 | PointCloud2 |
| `AiryPacketBridge` | 从结构化点云提取列 → 编码 MSOP | RslidarPacket @ 2250Hz |
| `AiryGpuLidarPlugin` | 2000Hz 逐块 GPU 渲染 (备选方案) | PointCloudPacked |

### 5.3 MSOP 编码关键
- 1248 字节/包 = 42B Header + 8×148B Block
- TwoInOneBlock 交叠: 偶数 block ring 0-47, 奇数 ring 48-95
- 距离分辨率 0.005m, 14-bit 编码
- IMU 合成: GT odometry 差分 → 线加速度注入

### 5.4 已知限制
- AiryLinkFollower 未在 World SDF 中启用
- 当前使用 SDF 注入的 gpu_lidar (10Hz 完整帧) + PacketBridge 而非逐块渲染
- 无单元测试
- 仅在默认场地 (rmuc_2025) 完整验证

---

## 第六部分: 怎么开始做开发 (5 分钟)

### 6.1 常见任务的入口

| 我想做... | 改哪里 | 参考 |
|----------|--------|------|
| 加新传感器 | `spawn_robots.launch.py` + `ros_gz_bridge.yaml` | 仿照 Airy 注入 |
| 写新 Gazebo 插件 | 新建包, 参考 `airy_gazebo_plugin/` | Layer 2 接口 |
| 消费传感器数据 | 订阅 `/<ns>/` 下的话题 | Layer 5 表格 |
| 跑 LIO 算法 | 启动 PacketBridge, 消费 `/lidar/packets` | AiryPacketBridge |
| 换场地测试 | 改 `gz_world.yaml` | Layer 0 |
| 双机器人对抗 | 取消注释蓝方配置 | Layer 0 |

### 6.2 环境搭建
```bash
git clone ... && vcs import && rosdep install && colcon build
# 或用 Docker:
docker run ghcr.io/smbu-polarbear-robotics-team/rmu_gazebo_simulator:1.0.0
```

---

## 第七部分: Q&A + 实操演示 (5-10 分钟)

### 建议演示
1. 启动仿真 → 展示 Gazebo GUI + RViz
2. `ros2 topic list` 展示所有话题
3. `ros2 topic echo /lidar/packets` 展示 MSOP 包流
4. 修改 `gz_world.yaml` 切换场地

---

## 备忘卡片

- **仓库地址**: SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator
- **当前分支**: `dev/airy-lidar-simulation` (我们的工作)
- **上游分支**: `main` (Lihan Chen)
- **接口文档**: `docs/interfaces.md`
- **设计文档**: `docs/superpowers/specs/2026-07-02-airy-lidar-simulation-design.md`
- **一键启动**: `scripts/sim.sh`
- **⚠️ 别忘了点 Gazebo 的 "启动" 按钮！**
