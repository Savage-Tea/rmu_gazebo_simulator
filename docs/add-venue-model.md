# 添加新场地模型（STP → Gazebo 场景）

从 CAD 的 STP 文件到 Gazebo 可加载场地的完整流程。

## 目标

- 输入：`~/Downloads/UTF-8__RMUC2026_V2.0.0.stp`（1.25 GB，文件名注意 UTF-8 前缀）
- 输出：`resource/models/rmuc_2026/` + `resource/worlds/rmuc_2026_world.sdf` + `gz_world.yaml` 注册

## 当前进度

- [x] Step 1 STP→STL 转换完成（`meshes/rmuc_2026.stl`，25 万面 / 12MB）
- [x] Step 2/3/4/5 全部就绪（model.config / model.sdf / rmuc_2026_world.sdf / gz_world.yaml 中 rmuc_2026 段）
- [ ] Step 5 `gz_world.yaml` 中 `world:` 切到 `rmuc_2026`（启动验证通过后）
- [ ] Step 6 启动验证

### 实际转换流程（2026-09-04 最终版，迭代后修正）

1. **pythonocc-core 读 STP**：`TransferRoot` 导入 ~2 min，292,566 面全部转换
2. **BRepMesh 曲面细分**：deflection=**5.0mm（毫米单位！）**、ang=0.1 → 790 万三角形 / 4.4 min
   （早期误按米传参 0.005 = 5µm，产生 6700 万三角形、2h+）
3. **流式写 STL**（mm，未平移）
4. **坐标修正**：原始地面在 z=-1941mm（模型原点在墙顶），平移 +1941mm 归零
5. **15mm 体素聚类**（numpy 位打包，流式）：790 万 → 37.3 万面，同时换算米制
6. **不经过 MeshLab 减面**——实测二次误差减面会把细杆/小结构整体塌缩丢失
   （P2/P3 附近 2m 高结构在 250k 与 1M 两次减面中均消失，聚类产物完好）
7. 最终：29.75×16×3.8m，37.3 万面 / 18MB，地面 480m²（大面保留），围墙+场地部件完整

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
mesh = importSTEP.open('/home/SavageTea/Downloads/UTF-8__RMUC2026_V2.0.0.stp')
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
