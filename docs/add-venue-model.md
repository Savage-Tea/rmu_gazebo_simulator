# 添加新场地模型（STP → Gazebo 场景）

从 CAD 的 STP 文件到 Gazebo 可加载场地的完整流程。

## 目标

- 输入：`~/Downloads/RMUC2026_V2.0.0.stp`
- 输出：`resource/models/rmuc_2026/` + `resource/worlds/rmuc_2026_world.sdf` + `gz_world.yaml` 注册

## Step 1 — STP → STL 转换

```bash
# 安装 FreeCAD (Arch)
sudo pacman -S --noconfirm freecad

# 创建目标目录
mkdir -p ~/Documents/RoboMaster/S26-27/Nav/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2026/meshes

# 转换 STP → STL
freecadcmd -c "
import Mesh
import importSTEP
mesh = importSTEP.open('/home/SavageTea/Downloads/RMUC2026_V2.0.0.stp')
Mesh.export([mesh], '/home/SavageTea/Documents/RoboMaster/S26-27/Nav/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2026/meshes/rmuc_2026.stl')
print('Done')
"
```

> **注意**：STP 的碰撞检测 mesh 需要简化（decimate 到 <500 面），否则物理引擎很慢。
> 如需材质颜色，改用 Blender 导出 DAE。

## Step 2 — 模型元数据

`resource/models/rmuc_2026/model.config`：

```xml
<?xml version="1.0"?>
<model>
  <name>rmuc_2026</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>SavageTea</name>
    <email>savage_tea@foxmail.com</email>
  </author>
  <description>RMUC 2026 competition venue</description>
</model>
```

## Step 3 — 模型 SDF

`resource/models/rmuc_2026/model.sdf`：

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="rmuc_2026">
    <static>true</static>
    <link name="body">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://rmuc_2026/meshes/rmuc_2026.stl</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://rmuc_2026/meshes/rmuc_2026.stl</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

## Step 4 — 世界 SDF

```bash
cp resource/worlds/rmuc_2025_world.sdf resource/worlds/rmuc_2026_world.sdf
```

把 `<uri>model://rmuc_2025</uri>` 改成 `<uri>model://rmuc_2026</uri>`，`<model name='rmuc_2025'>` 改成 `rmuc_2026`。

必须保留：
- `<physics><max_step_size>0.0005</max_step_size></physics>`
- `<render_engine>ogre2</render_engine>`
- `<light>` 光源

## Step 5 — 注册机器人

`config/gz_world.yaml`：

```yaml
world: "rmuc_2026"    # 切换默认世界

robots:
  rmuc_2026:
    - name: "red_standard_robot1"
      color: "red"
      x_pose: "0.0"     # 场地中心 / 起点
      y_pose: "0.0"
      z_pose: "0.28"
      yaw: "0.0"
```

## Step 6 — 启动验证

```bash
colcon build --symlink-install --packages-select rmu_gazebo_simulator
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

检查场地模型在 Gazebo GUI 中正确显示，无碰撞警告。
