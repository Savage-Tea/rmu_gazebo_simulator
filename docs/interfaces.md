# RMU Gazebo Simulator — 接口规格书

> 维护者: SavageTea | 日期: 2026-07-05 | 分支: `dev/airy-lidar-simulation`

## 概述

本文档以**接口契约**为核心视角，描述 `rmu_gazebo_simulator` 各组件之间的输入/输出、消息类型、命名约定和配置格式。面向需要在此仿真环境中进行二次开发的团队成员。

### 接口层次

```
┌──────────────────────────────────────────────────────────────────────┐
│  Layer 5: ROS 2 应用接口   ← 算法开发者最常接触                        │
│  Layer 4: ros_gz_bridge    ← Gazebo ↔ ROS 协议转换                    │
│  Layer 3: Gazebo Transport ← 仿真内部消息总线                          │
│  Layer 2: ECS 组件接口     ← 实体状态读写                              │
│  Layer 1: SDF 配置接口     ← World/Model/Sensor 声明式配置             │
│  Layer 0: Launch 组装接口  ← 启动时组装所有组件                         │
└──────────────────────────────────────────────────────────────────────┘
```

### 数据流全景

```
World SDF (0.5ms 物理步长)
  │
  ├─→ Gazebo Sensors (Ogre2)
  │     ├─→ Mid360 LiDAR/IMU ──→ ros_gz_bridge ──→ /<ns>/livox/*
  │     ├─→ RPLidar A2        ──→ ros_gz_bridge ──→ /<ns>/rplidar_a2/scan
  │     ├─→ 工业相机            ──→ ros_gz_bridge ──→ /<ns>/front_industrial_camera/*
  │     └─→ Airy GpuLidar      ──→ ros_gz_bridge ──→ /<ns>/airy/lidar
  │                                                        │
  │                                                        ▼
  │                                                AiryPacketBridge
  │                                                        │
  │                                          ┌─────────────┴──────────────┐
  │                                          ▼                            ▼
  │                                  /lidar/packets                  /lidar/imu
  │                               (RslidarPacket)             (sensor_msgs/Imu)
  │                                          │                            │
  │                                          └────────────┬───────────────┘
  │                                                       ▼
  │                                              XSmall Point-LIO
  │                                                       │
  │                                                       ▼
  │                                                  /Odometry
  │                                                       │
  └──────────────────→ /<ns>/chassis_odometry_gt ←─── evo 定量对比
```

---

## Layer 0: Launch 组装接口

### 主入口: `bringup_sim.launch.py`

**路径**: `rmu_gazebo_simulator/launch/bringup_sim.launch.py`

**输入参数**: 无命令行参数，所有配置从 `config/gz_world.yaml` 读取。

**组装流程**:

```
bringup_sim.launch.py
  │
  ├─(1)─→ gazebo.launch.py
  │       参数: world_sdf_path, ign_config_path
  │       └─→ ros_gz_sim/gz_sim.launch.py (gz_version=6)
  │           输出: Gazebo Server + GUI
  │
  ├─(2)─→ spawn_robots.launch.py
  │       参数: gz_world_path, world
  │       对每个 robot:
  │         ├─ xmacro 展开 → SDF 字符串
  │         ├─ XML 注入 Airy 传感器到 front_mid360 link
  │         ├─ SDF→URDF 转换 (sdformat_tools.UrdfGenerator)
  │         ├─ ros_gz_sim create (生成机器人)
  │         ├─ rmua19_robot_base (底盘/云台控制)
  │         ├─ robot_state_publisher (TF)
  │         ├─ ros_gz_bridge parameter_bridge (传感器桥接)
  │         └─ ign service set_performer (Level 系统注册)
  │
  └─(3)─→ referee_system.launch.py
           └─→ simple_competition_1v1.py (裁判逻辑)
```

### 关键命名约定

| 约定 | 值 | 说明 |
|------|-----|------|
| 机器人命名 | `{color}_standard_robot{N}` | 如 `red_standard_robot1`, `blue_standard_robot1` |
| ROS 2 命名空间 | 与机器人名相同 | 所有传感器话题在命名空间下 |
| 世界命名 | `{arena}_{year}_world.sdf` | 如 `rmuc_2025_world.sdf` |
| 世界名称 | `default` | 所有 World SDF 的统一 name 属性 |

### 场地选择接口

修改 `rmu_gazebo_simulator/config/gz_world.yaml`:

```yaml
world: "rmuc_2025"  # 可选: rmul_2024, rmuc_2024, rmul_2025, empty_world

robots:
  rmuc_2025:
    - name: "red_standard_robot1"
      color: "red"
      x_pose: "3.4"
      y_pose: "9.5"
      z_pose: "0.28"
      yaw: "0.0"
    # 取消注释以添加蓝方机器人:
    # - name: "blue_standard_robot1"
    #   color: "blue"
    #   x_pose: "25.6"
    #   y_pose: "6.45"
    #   z_pose: "0.28"
    #   yaw: "3.14"
```

---

## Layer 1: SDF 配置接口

### 1.1 World SDF 契约

**路径**: `rmu_gazebo_simulator/resource/worlds/{world}_world.sdf`

每个 World SDF 必须声明以下插件才能正常运行：

```xml
<sdf version="1.7">
  <world name="default">

    <!-- ═══════════════════════════════════════════════════════════════ -->
    <!-- 物理引擎配置 (Airy LiDAR 要求 2000Hz+)                         -->
    <!-- ═══════════════════════════════════════════════════════════════ -->
    <physics type="ode">
      <max_step_size>0.0005</max_step_size>        <!-- 0.5ms = 2000Hz -->
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>2000</real_time_update_rate>
    </physics>

    <!-- ═══════════════════════════════════════════════════════════════ -->
    <!-- 标准 Gazebo 系统插件 (缺一不可)                                -->
    <!-- ═══════════════════════════════════════════════════════════════ -->
    <plugin name="ignition::gazebo::systems::Physics"
            filename="libignition-gazebo-physics-system.so"/>
    <plugin name="ignition::gazebo::systems::SceneBroadcaster"
            filename="libignition-gazebo-scene-broadcaster-system.so"/>
    <plugin name="ignition::gazebo::systems::UserCommands"
            filename="libignition-gazebo-user-commands-system.so"/>
    <plugin name="ignition::gazebo::systems::Sensors"
            filename="libignition-gazebo-sensors-system.so">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="ignition::gazebo::systems::Imu"
            filename="libignition-gazebo-imu-system.so"/>

    <!-- ═══════════════════════════════════════════════════════════════ -->
    <!-- 自定义插件: Airy GPU LiDAR (可选)                              -->
    <!-- ═══════════════════════════════════════════════════════════════ -->
    <plugin name="airy_gazebo_plugin::AiryGpuLidarPlugin"
            filename="libairy_gazebo_plugin.so">
      <robot_name>red_standard_robot1</robot_name>
      <lidar_link>front_mid360</lidar_link>
      <max_range>60.0</max_range>
      <min_range>0.1</min_range>
    </plugin>

    <!-- ═══════════════════════════════════════════════════════════════ -->
    <!-- 场地模型                                                       -->
    <!-- ═══════════════════════════════════════════════════════════════ -->
    <model name="rmuc_2025">
      <include><uri>model://rmuc_2025</uri></include>
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
    </model>

    <!-- 光照 -->
    <light type="directional" name="direct_light">
      <cast_shadows>true</cast_shadows>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>0 0 -1</direction>
    </light>

  </world>
</sdf>
```

### 1.2 场地模型约定

```
model://{arena_name}          # 模型 URI (由 IGN_GAZEBO_RESOURCE_PATH 解析)
  static=true                 # 静态模型
  碰撞网格: ODE friction 表面
  视觉网格: STL (2024) 或 DAE+纹理 (2025)
```

**可用场地**:

| 场地 | 模型 URI | 网格格式 | 特点 |
|------|---------|---------|------|
| RMUL 2024 | `model://rmul_2024` | STL (scale 0.001) | 含物理属性、供应商区域 |
| RMUC 2024 | `model://rmuc_2024` | STL | 含物理属性、地面 |
| RMUL 2025 | `model://rmul_2025` | DAE + JPG | 纯视觉/碰撞、无地面 |
| RMUC 2025 | `model://rmuc_2025` | STL | 纯视觉/碰撞、带材质 |

### 1.3 传感器注入接口

在 `spawn_robots.launch.py` 中，通过 XML 操作将此传感器注入到每个机器人的 `front_mid360` link：

```xml
<sensor type="gpu_lidar" name="front_airy_sensor">
  <ignition_frame_id>front_mid360</ignition_frame_id>  <!-- 坐标系 -->
  <pose>0 0 0.06 0 0 0</pose>                          <!-- link 内偏移 -->
  <always_on>true</always_on>
  <update_rate>10</update_rate>                         <!-- 10Hz 完整帧 -->

  <ray>
    <scan>
      <horizontal>
        <samples>900</samples>                          <!-- 360°/0.4° -->
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>96</samples>                           <!-- 96 线 -->
        <min_angle>-0.785398</min_angle>                <!-- -45° -->
        <max_angle>0.785398</max_angle>                 <!-- +45° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>60.0</max>
      <resolution>0.005</resolution>                    <!-- 5mm -->
    </range>
  </ray>

  <noise>
    <type>gaussian</type>
    <stddev>0.01</stddev>                               <!-- σ = 1cm -->
  </noise>
</sensor>
```

### 1.4 独立 Airy LiDAR 模型

**路径**: `rmu_gazebo_simulator/resource/models/airy_lidar/model.sdf`

仅在 `rmuc_2025_world.sdf` 中使用，作为独立静态模型放置在机器人出生点：

```xml
<model name="airy_lidar">
  <include><uri>model://airy_lidar</uri></include>
  <pose>3.4 9.5 0.28 0 0 0</pose>   <!-- 与机器人出生点相同 -->
</model>
```

此独立模型需要 `AiryLinkFollower` 插件来跟踪机器人移动（当前未启用）。

---

## Layer 2: ECS 组件接口 (Gazebo System Plugin 开发)

### 2.1 插件注册契约

每个 Gazebo System Plugin 必须按如下模式定义：

```cpp
#include <ignition/gazebo/System.hh>

namespace my_namespace {

class MyPlugin : public ignition::gazebo::System,
                 public ignition::gazebo::ISystemConfigure,    // 初始化
                 public ignition::gazebo::ISystemPostUpdate    // 每步执行
{
public:
  MyPlugin() = default;

  /// @brief 只调用一次，在仿真开始前
  /// @param entity  插件关联的实体 (世界插件则为世界实体)
  /// @param sdf     指向此插件 SDF 元素的指针
  /// @param ecm     实体组件管理器 (可读写)
  /// @param eventMgr 事件管理器
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager & eventMgr) override;

  /// @brief 每个仿真步调用一次 (2000Hz @ 0.5ms 步长)
  /// @param info  仿真步信息 (时间戳、步长等)
  /// @param ecm   实体组件管理器 (const, 写入需 const_cast)
  void PostUpdate(
    const ignition::gazebo::UpdateInfo & info,
    const ignition::gazebo::EntityComponentManager & ecm) override;
};

}  // namespace my_namespace

// 注册为 Gazebo 插件
IGNITION_ADD_PLUGIN(
  my_namespace::MyPlugin,
  ignition::gazebo::System,
  my_namespace::MyPlugin::ISystemConfigure,
  my_namespace::MyPlugin::ISystemPostUpdate
)
```

### 2.2 ECS 组件读取接口

```cpp
// 遍历所有带特定组件的实体：
ecm.Each<ignition::gazebo::components::Name,
         ignition::gazebo::components::WorldPose>(
  [&](const ignition::gazebo::Entity & ent,
      const ignition::gazebo::components::Name * name,
      const ignition::gazebo::components::WorldPose * pose) -> bool
  {
    if (name->Data() == "front_mid360") {
      auto pos = pose->Data();    // ignition::math::Pose3d
      return false;               // 停止遍历
    }
    return true;                  // 继续遍历
  });
```

### 2.3 ECS 组件写入接口

```cpp
// Configure() 中可以直接写：
ecm.CreateComponent<ignition::gazebo::components::WorldPose>(entity, newPose);

// PostUpdate() 中 ECM 是 const 的，写入需 const_cast：
auto * comp = const_cast<ignition::gazebo::EntityComponentManager &>(ecm)
                .Component<ignition::gazebo::components::WorldPose>(entity);
if (comp) {
  *comp = ignition::gazebo::components::WorldPose(newPose);
}
```

### 2.4 关键 ECS 组件类型

| 组件类型 | 包含数据 | 典型用途 |
|---------|---------|---------|
| `components::WorldPose` | `ignition::math::Pose3d` (位置+姿态) | 读取/写入实体世界位姿 |
| `components::Name` | `std::string` | 按名称匹配查找目标实体 |
| `components::ParentEntity` | `Entity` (父实体 ID) | 层级关系查找 |

### 2.5 延迟查找模式 (Fix #1)

Configure 时实体可能尚未生成 → 在 PostUpdate 中延迟查找：

```cpp
bool entity_found_ = false;
ignition::gazebo::Entity target_entity_{ignition::gazebo::kNullEntity};

void Configure(...) {
  entity_found_ = false;  // 标记未找到
}

void PostUpdate(const UpdateInfo &, const ECM & ecm) {
  // 延迟查找：每次 PostUpdate 重试直到找到
  if (!entity_found_) {
    ecm.Each<components::Name, components::ParentEntity>(
      [&](const Entity & ent, const components::Name * name,
          const components::ParentEntity *) {
        if (name->Data() == target_link_name_) {
          target_entity_ = ent;
          entity_found_ = true;
          return false;
        }
        return true;
      });
    if (!entity_found_) return;  // 还没生成，等下次
  }

  // 正常业务逻辑...
}
```

---

## Layer 3: Gazebo Transport 接口 (仿真内部消息)

### 3.1 Topic 命名约定

```
/world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}/scan/points
```

**具体实例**:

| 传感器 | Gazebo Transport Topic |
|--------|----------------------|
| AiryGpuLidarPlugin 输出 | `/world/default/model/red_standard_robot1/link/front_mid360/sensor/airy_lidar/scan/points` |
| SDF 注入 Airy 传感器输出 | `/world/default/model/red_standard_robot1/link/front_mid360/sensor/front_airy_sensor/scan/points` |
| Mid360 LiDAR 输出 | `/world/default/model/red_standard_robot1/link/front_mid360/sensor/front_mid360_lidar/scan/points` |
| Mid360 IMU 输出 | `/world/default/model/red_standard_robot1/link/front_mid360/sensor/front_mid360_imu/imu` |
| RPLidar A2 输出 | `/world/default/model/red_standard_robot1/link/front_rplidar_a2/sensor/front_rplidar_a2/scan` |
| 工业相机输出 | `/world/default/model/red_standard_robot1/link/front_industrial_camera/sensor/front_industrial_camera/image` |
| 里程计输出 | `/red_standard_robot1/odometry` |

### 3.2 常用消息类型

| 传感器类型 | Gazebo 消息类型 |
|-----------|----------------|
| LiDAR 点云 | `ignition::msgs::PointCloudPacked` |
| 2D 激光 | `ignition::msgs::LaserScan` |
| 里程计 | `ignition::msgs::Odometry` |
| IMU | `ignition::msgs::IMU` |
| 图像 | `ignition::msgs::Image` |
| 相机内参 | `ignition::msgs::CameraInfo` |
| 关节状态 | `ignition::msgs::Model` |

### 3.3 发布器生命周期管理 (Fix #2)

```cpp
// 用 shared_ptr 持有 Node, pub_ 的生命周期与 Node 相同
std::shared_ptr<ignition::transport::Node> ign_node_;  // 类成员
ignition::transport::Node::Publisher pub_;              // 类成员

void Configure(...) {
  ign_node_ = std::make_shared<ignition::transport::Node>();
  pub_ = ign_node_->Advertise<ignition::msgs::PointCloudPacked>(topic);
  // Node 不会在 Configure 返回后析构 → pub_ 始终有效
}
```

---

## Layer 4: ros_gz_bridge 接口 (Gazebo ↔ ROS 2)

### 4.1 Bridge 配置格式

**路径**: `rmu_gazebo_simulator/config/ros_gz_bridge.yaml`

每一行的配置契约：

```yaml
- ros_topic_name: "/<robot_name>/<topic>"          # ROS 2 侧话题
  gz_topic_name: "/world/default/model/<robot_name>/link/<link>/sensor/<sensor>/<data>"
  ros_type_name: "sensor_msgs/msg/PointCloud2"      # ROS 2 消息类型
  gz_type_name: "ignition.msgs.PointCloudPacked"     # Gazebo 消息类型
  direction: "GZ_TO_ROS"                             # GZ_TO_ROS | ROS_TO_GZ | BIDIRECTIONAL
```

`<robot_name>` 是占位符，由 `spawn_robots.launch.py` 在运行时根据机器人名替换。

### 4.2 当前桥接映射表

| ROS 2 Topic (相对命名空间) | Gazebo Topic (相对) | ROS 类型 | Gazebo 类型 | 方向 |
|---|---|---|---|---|
| `chassis_odometry_gt` | `/<ns>/odometry` | `nav_msgs/Odometry` | `ignition.msgs.Odometry` | GZ→ROS |
| `joint_states` | `/world/default/model/<ns>/joint_state` | `sensor_msgs/JointState` | `ignition.msgs.Model` | GZ→ROS |
| `front_industrial_camera/image` | `.../front_industrial_camera/sensor/.../image` | `sensor_msgs/Image` | `ignition.msgs.Image` | GZ→ROS |
| `front_industrial_camera/camera_info` | `.../camera_info` | `sensor_msgs/CameraInfo` | `ignition.msgs.CameraInfo` | GZ→ROS |
| `rplidar_a2/scan` | `.../front_rplidar_a2/sensor/.../scan` | `sensor_msgs/LaserScan` | `ignition.msgs.LaserScan` | GZ→ROS |
| `livox/lidar` | `.../front_mid360/sensor/front_mid360_lidar/scan/points` | `sensor_msgs/PointCloud2` | `ignition.msgs.PointCloudPacked` | GZ→ROS |
| `livox/imu` | `.../front_mid360/sensor/front_mid360_imu/imu` | `sensor_msgs/Imu` | `ignition.msgs.IMU` | GZ→ROS |
| `airy/lidar` | `.../front_mid360/sensor/front_airy_sensor/scan/points` | `sensor_msgs/PointCloud2` | `ignition.msgs.PointCloudPacked` | GZ→ROS |

### 4.3 时钟桥接

在 `gazebo.launch.py` 中单独配置：

```bash
ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

---

## Layer 5: ROS 2 应用接口 (算法开发层)

### 5.1 传感器数据输出 (GZ→ROS, 只读)

所有话题都在机器人命名空间下 (如 `/red_standard_robot1/`):

| Topic | 类型 | 频率 | 用途 |
|---|---|---|---|
| `/chassis_odometry_gt` | `nav_msgs/Odometry` | ~2000Hz | **真值里程计** (算法评估基准) |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10Hz | Mid360 导航点云 |
| `/livox/imu` | `sensor_msgs/Imu` | ~1000Hz | Mid360 IMU |
| `/airy/lidar` | `sensor_msgs/PointCloud2` | 10Hz | **Airy 96线** 结构化点云 (900×96) |
| `/rplidar_a2/scan` | `sensor_msgs/LaserScan` | ~10Hz | 2D 激光扫描 |
| `/front_industrial_camera/image` | `sensor_msgs/Image` | ~30Hz | 前视相机 |
| `/front_industrial_camera/camera_info` | `sensor_msgs/CameraInfo` | ~30Hz | 相机内参 |
| `/joint_states` | `sensor_msgs/JointState` | ~50Hz | 关节角度 |

### 5.2 机器人控制输入 (ROS→GZ, 写入)

控制通过 `rmoss_gz_base` 包的 `rmua19_robot_base` 节点实现：

```bash
# 底盘移动
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args \
  -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3

# 云台控制
ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args \
  -r __ns:=/red_standard_robot1/robot_base

# 射击控制
ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args \
  -r __ns:=/red_standard_robot1/robot_base
```

### 5.3 控制参数接口

**路径**: `rmu_gazebo_simulator/config/base_params.yaml`

```yaml
world_name: "default"
chassis_controller:
  follow_yaw:
    pid: { p: 5.0, i: 0.0, d: 1.0 }
gimbal_controller:
  pitch:
    pid: { p: 10.0, i: 0.1, d: 1.0 }
  yaw:
    pid: { p: 10.0, i: 0.0, d: 1.0 }
```

---

## AiryPacketBridge 接口规格

### 节点信息

| 属性 | 值 |
|------|-----|
| 节点名 | `airy_packet_bridge_node` |
| 类型 | `rclcpp::Node` (支持 `rclcpp_components` 组件加载) |
| 命名空间 | 可配置 (默认 `red_standard_robot1`) |
| 启动文件 | `airy_packet_bridge/launch/airy_packet_bridge.launch.py` |

### 输入接口 (订阅)

| Topic | 类型 | 频率 | 用途 |
|---|---|---|---|
| `/<ns>/airy/lidar` | `sensor_msgs/PointCloud2` | 10Hz | 完整 900×96 结构化点云帧 |
| `/<ns>/livox/imu` | `sensor_msgs/Imu` | ~1000Hz | IMU 透传源 |
| `/<ns>/chassis_odometry_gt` | `nav_msgs/Odometry` | ~2000Hz | GT 里程计 (用于合成线加速度) |

### 输出接口 (发布)

| Topic | 类型 | 频率 | 用途 |
|---|---|---|---|
| `/lidar/packets` | `rslidar_msg/RslidarPacket` | ~2250Hz | Airy MSOP 数据包流 |
| `/lidar/imu` | `sensor_msgs/Imu` | ~1000Hz | 合成 IMU (透传 + 加速度) |

> **注意**: 输出话题在**全局命名空间**下 (无 `/ns` 前缀)，因为 XSmall Point-LIO 期望全局话题。

### 参数

**路径**: `airy_packet_bridge/config/airy_packet_bridge.yaml`

```yaml
robot_name: "red_standard_robot1"   # 目标机器人名
cloud_topic: "airy/lidar"           # 输入点云话题 (相对于命名空间)
imu_topic: "livox/imu"              # 输入 IMU 话题 (相对于命名空间)
use_sim_time: true                  # 使用仿真时间
```

### MSOP 包格式 (输出契约)

供下游 `rs_driver` / XSmall Point-LIO 解码：

```
1248 字节/包
├── 42 字节 Header
│   ├── 4B Magic:  {0x55, 0xAA, 0x05, 0x5A}
│   └── UTC 时间戳 (μs + s)
└── 8 × 148 字节 Block
    ├── 2B Block Magic: {0xFF, 0xEE}
    ├── 2B Azimuth (deg × 100, uint16 big-endian)
    └── 48 × 3B Channel
        ├── 14-bit Distance (0.005m 分辨率, max 0x3FFF)
        ├── 2-bit Feature Flags (仿真中恒为 0)
        └── 1B Intensity (仿真中恒为 0)

Block 交叠模式:
  偶数 Block (0,2,4,6): ring 0-47
  奇数 Block (1,3,5,7): ring 48-95
  每 2 Block 共享 1 个 azimuth 样本 (TwoInOneBlockIterator)

每圈包数: 225 包
每包 azimuth 组: 4 个 (8 blocks / 2)
```

### IMU 合成约定

```
正常情况: 发布透传 IMU (含重力分量)
加速度事件: GT odom 线速度差分 |dv/dt| > 0.01 m/s²
           → 发布合成 IMU (角速度 + 线加速度分量)
丢包检测: 相邻包 azimuth 间隔 > 5°
         → 重置 ring buffer
```

---

## AiryGpuLidarPlugin 接口规格

### 插件信息

| 属性 | 值 |
|------|-----|
| 类名 | `airy_gazebo_plugin::AiryGpuLidarPlugin` |
| 类型 | Gazebo System Plugin (世界级) |
| 加载方式 | World SDF 中的 `<plugin>` 元素 |
| 共享库 | `libairy_gazebo_plugin.so` |

### SDF 配置参数

```xml
<plugin name="airy_gazebo_plugin::AiryGpuLidarPlugin"
        filename="libairy_gazebo_plugin.so">
  <robot_name>red_standard_robot1</robot_name>   <!-- 目标机器人名 -->
  <lidar_link>front_mid360</lidar_link>           <!-- LiDAR 安装 link -->
  <max_range>60.0</max_range>                     <!-- 最大测距 (m) -->
  <min_range>0.1</min_range>                      <!-- 最小测距 (m) -->
</plugin>
```

### 输出接口

| 属性 | 值 |
|------|-----|
| Topic | `/world/default/model/{robot_name}/link/{lidar_link}/sensor/airy_lidar/scan/points` |
| 消息类型 | `ignition::msgs::PointCloudPacked` |
| 频率 | ~2250 Hz (每 block 发布一次) |
| 点数/消息 | 96 点 (每条垂直射线一个) |

### 渲染参数常量

```cpp
static constexpr int kNumLines = 96;                  // 96 线
static constexpr double kRpm = 600.0;                 // 600 RPM
static constexpr double kDegPerSec = 3600.0;          // 3600°/s
static constexpr double kBlockDurationUs = 111.080;   // 111.08 μs/block
static constexpr double kAzimuthPerBlock = ~0.4;      // ~0.4°/block
```

---

## AiryLinkFollower 接口规格

### 插件信息

| 属性 | 值 |
|------|-----|
| 类名 | `airy::AiryLinkFollower` |
| 类型 | Gazebo System Plugin (世界级) |
| 共享库 | `libairy_link_follower.so` |
| 状态 | 已编译但未在任何 World SDF 中启用 |

### SDF 配置参数

```xml
<plugin name="airy::AiryLinkFollower" filename="libairy_link_follower.so">
  <target_model>red_standard_robot1</target_model>   <!-- 目标机器人 -->
  <target_link>front_mid360</target_link>             <!-- 跟随目标 link -->
  <follower_link>airy_lidar::airy_link</follower_link> <!-- 被移动的 link -->
</plugin>
```

### 功能

每仿真步复制 `target_link` 的 `WorldPose` 到 `follower_link`，用于将独立 Airy LiDAR 模型锁定到移动机器人上。

---

## 裁判系统接口

### 参数

**路径**: `rmu_gazebo_simulator/config/referee_system_1v1.yaml`

```yaml
max_hp: 500
initial_projectiles: 100
initial_resources: 200
```

### 状态机

```
PREPARATION ──→ SELF_CHECKING ──→ START_GAME ──→ STOP_GAME
                                        │             │
                    KILL_ROBOT ←────────┘             │
                    REVIVE_ROBOT ─────────────────────┘
```

### 规则

| 规则 | 数值 |
|------|------|
| 装甲板命中 | -10 HP |
| 超速弹丸 (>30m/s) | 自伤 -10 HP |
| HP ≤ 0 | 机器人自动击毁 |
| 资源回复 | +50/队/30秒 |

### 网页端

| 端 | 地址 | 启动命令 |
|----|------|---------|
| 操作手 | `http://localhost:5000` | `python3 scripts/player_web/main_no_vision.py` |
| 裁判系统 | `http://localhost:2350` | `python3 scripts/referee_web/main.py` |

---

## 环境变量接口

**路径**: `rmu_gazebo_simulator/env-hooks/gazebo.dsv.in`

每次 `source install/setup.sh` 时自动设置：

```bash
export GAZEBO_PLUGIN_PATH=.../lib:$GAZEBO_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=.../resource/models:.../resource/worlds:$IGN_GAZEBO_RESOURCE_PATH
export SDF_PATH=.../resource/models:.../resource/worlds:$SDF_PATH
export IGN_FILE_PATH=.../resource/models:.../resource/worlds:$IGN_FILE_PATH
export MESA_GL_VERSION_OVERRIDE=3.3   # Ogre2 兼容性关键！
```

---

## 接口开发速查表

### 场景 A: 添加新传感器到机器人

| 步骤 | 文件 | 修改内容 |
|------|------|---------|
| 1 | `spawn_robots.launch.py` | 仿照 Airy 注入代码，添加 sensor XML 到目标 link |
| 2 | `config/ros_gz_bridge.yaml` | 添加 `ros_topic → gz_topic` 映射行 |
| 3 | (可选) 新写插件 | 如果需要自定义 GPU 渲染逻辑 |

### 场景 B: 消费仿真数据做算法开发

| 数据 | 话题 | 命名空间 |
|------|------|---------|
| Ground Truth 里程计 | `/<ns>/chassis_odometry_gt` | 机器人名 |
| Airy 点云 | `/<ns>/airy/lidar` | 机器人名 |
| Airy MSOP 包 | `/lidar/packets` | **全局** (无命名空间) |
| IMU | `/lidar/imu` | **全局** (无命名空间) |

### 场景 C: 更换仿真场地

1. 修改 `config/gz_world.yaml` → `world: "目标场地"`
2. 确认该场地的机器人出生点已配置
3. 确认 World SDF 文件存在于 `resource/worlds/`

### 场景 D: 添加蓝方机器人

在 `config/gz_world.yaml` 的目标场地下取消注释蓝方条目。

### 场景 E: 编写新的 Gazebo 插件

```cmake
# CMakeLists.txt
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_library(my_plugin SHARED src/my_plugin.cpp)
target_link_libraries(my_plugin
  ignition-gazebo6::ignition-gazebo6
  ignition-rendering6::ignition-rendering6
  ignition-transport11::ignition-transport11)
```

```cpp
// 必须遵循的接口契约见 Layer 2
// 注册宏:
IGNITION_ADD_PLUGIN(my_namespace::MyPlugin, ignition::gazebo::System, ...)
```

---

## 依赖仓库

| 仓库 | 用途 |
|------|------|
| `sdformat_tools` | SDF ↔ URDF 转换、xmacro 处理 |
| `rmoss_interfaces` | 裁判系统 ROS 消息 |
| `rmoss_core` | RoboMaster 仿真核心库 |
| `rmoss_gazebo` | 机器人控制 Gazebo 插件 |
| `rmoss_gz_resources` | 机器人模型资源 |
| `pb2025_robot_description` | 机器人 SDF xmacro 模板 |
| `rslidar_msg` | RoboSense RslidarPacket 消息定义 |

## 注意事项

1. **Gazebo "启动"按钮**: 仿真默认暂停，必须点击 GUI 左下角橙红色按钮
2. **物理步长 0.5ms**: 为 Airy 2000Hz 渲染需求修改，影响所有 World SDF
3. **MESA_GL_VERSION_OVERRIDE=3.3**: Ogre2 渲染引擎需要此环境变量
4. **延迟实体查找**: 机器人由 `ros_gz_sim create` 动态生成，插件 Configure 时可能尚未存在
5. **无测试**: CI 跳过测试，源码中无单元测试/集成测试文件
6. **两种 Airy LiDAR 部署方式**: World Plugin (2024 + rmul_2025) vs 独立模型 (rmuc_2025)
