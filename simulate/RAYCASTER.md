# RayCaster功能集成说明

本项目已成功集成mujoco_ray_caster的raycaster传感器功能，可以在MuJoCo仿真中使用激光雷达和深度相机，并通过独立的ROS2节点发布点云数据。

## 项目结构

```
simulate/
├── raycaster_src/          # RayCaster核心源代码
├── raycaster_plugin/       # MuJoCo插件实现
├── raycaster_ros2/         # 独立ROS2节点（详见raycaster_ros2/README.md）
│   ├── raycaster_ros2_node.h/cpp
│   ├── CMakeLists.txt & package.xml
│   ├── build.sh & run_simple.sh
│   ├── launch/raycaster.launch.py
│   ├── models/raycaster_test.xml
│   └── README.md           # 详细使用文档
└── CMakeLists.txt          # 已修改支持raycaster编译
```

## 快速开始

### 1. 编译

```bash
# 编译主项目（包含raycaster插件）
cd ~/clone/unitree_mujoco/simulate/build
cmake ..
make -j$(nproc)

# 编译ROS2节点
cd ~/clone/unitree_mujoco/simulate/raycaster_ros2
source /opt/ros/humble/setup.bash  # 根据你的ROS2版本
./build.sh
```

### 2. 运行

**方式1: 独立ROS2节点测试**
```bash
cd ~/clone/unitree_mujoco/simulate/raycaster_ros2
./run_simple.sh
```

**方式2: 与主仿真配合使用（推荐）**

终端1 - 主仿真（机器人控制）:
```bash
cd ~/clone/unitree_mujoco/simulate/build
./unitree_mujoco
```

终端2 - ROS2节点（传感器数据发布）:
```bash
cd ~/clone/unitree_mujoco/simulate/raycaster_ros2
source ros2_ws/install/setup.bash
ros2 run raycaster_ros2 raycaster_ros2_node --ros-args \
  -p model_file:=/path/to/your/model.xml
```

### 3. 查看数据

```bash
# 列出topic
ros2 topic list

# 查看点云
ros2 topic echo /raycaster/<sensor_name>

# 使用RViz2可视化
rviz2
```

## 在模型中添加RayCaster传感器

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.sensor.ray_caster"/>
  </extension>

  <worldbody>
    <body name="robot">
      <camera name="depth_camera" pos="0 0 0.5" quat="1 0 0 0"/>
    </body>
  </worldbody>

  <sensor>
    <plugin name="my_raycaster" plugin="mujoco.sensor.ray_caster" 
            objtype="camera" objname="depth_camera">
      <config key="resolution" value="0.1"/>
      <config key="size" value="1.57 0.785"/>
      <config key="dis_range" value="0.1 10"/>
      <config key="type" value="world"/>
      <config key="sensor_data_types" value="pos_w"/>
    </plugin>
    
    <!-- 可选：用于TF发布 -->
    <framepos name="depth_camera_pos" objtype="camera" objname="depth_camera"/>
    <framequat name="depth_camera_quat" objtype="camera" objname="depth_camera"/>
  </sensor>
</mujoco>
```

### 配置参数说明

- `resolution`: 角分辨率（弧度），值越小点云越密集
- `size`: 视场 `[水平FOV, 垂直FOV]`（弧度）
- `dis_range`: 测距范围 `[最小, 最大]`（米）
- `type`: 坐标系类型 `base`, `yaw`, 或 `world`（必需）
- `sensor_data_types`: 输出数据类型，常用 `pos_w`（世界坐标点）

## ROS2功能

- **Topic**: `/raycaster/<sensor_name>` (sensor_msgs/PointCloud2)
- **TF**: 自动发布相机坐标系变换（如果配置了相应传感器）
- **参数**:
  - `model_file` (必需): MuJoCo模型文件路径
  - `publish_rate` (默认: 30.0): 发布频率(Hz)
  - `publish_tf` (默认: true): 是否发布TF

## 设计特点

1. **完全解耦**: ROS2节点独立运行，不影响主仿真程序
2. **自动发现**: 自动检测模型中的所有raycaster传感器
3. **标准接口**: 使用ROS2标准PointCloud2消息格式
4. **灵活部署**: 可单独使用或配合主程序使用

## 详细文档

- [ROS2节点详细说明](raycaster_ros2/README.md)

## 参考资料

- 原始项目: https://github.com/Albusgive/mujoco_ray_caster
- MuJoCo文档: https://mujoco.readthedocs.io/
- ROS2文档: https://docs.ros.org/
