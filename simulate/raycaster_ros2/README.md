# MuJoCo RayCaster ROS2 Publisher

独立的ROS2节点，用于发布MuJoCo raycaster传感器的点云数据。

## 功能特性

- 独立运行，不依赖unitree_sdk2
- 自动检测模型中的所有raycaster传感器
- 发布PointCloud2格式的点云数据
- 可选发布TF变换
- 支持多个raycaster传感器同时发布

## 快速开始

### 1. 安装依赖

```bash
sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-tf2-ros libeigen3-dev
```

### 2. 编译

```bash
cd ~/clone/unitree_mujoco/simulate/raycaster_ros2
source /opt/ros/humble/setup.bash  # 或你的ROS2版本
./build.sh
```

### 3. 运行

**使用测试模型**:
```bash
./run_simple.sh
```

**使用自己的模型**:
```bash
source ros2_ws/install/setup.bash
ros2 run raycaster_ros2 raycaster_ros2_node --ros-args \
  -p model_file:=/path/to/your/model.xml \
  -p publish_rate:=30.0 \
  -p publish_tf:=true
```

**使用launch文件**:
```bash
source ros2_ws/install/setup.bash
ros2 launch raycaster_ros2 raycaster.launch.py \
  model_file:=/path/to/your/model.xml
```

### 4. 查看数据

```bash
# 列出所有topic
ros2 topic list

# 查看点云数据
ros2 topic echo /raycaster/<sensor_name>

# 查看发布频率
ros2 topic hz /raycaster/<sensor_name>

# 使用RViz2可视化
rviz2
```

## 参数配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| model_file | string | (必需) | MuJoCo模型文件路径 |
| publish_rate | double | 30.0 | 发布频率(Hz) |
| publish_tf | bool | true | 是否发布TF变换 |

## 发布的Topic

- `/raycaster/<sensor_name>` - sensor_msgs/PointCloud2: 点云数据
- `/tf` - TF变换（如果publish_tf=true）

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
    <!-- RayCaster传感器 -->
    <plugin name="my_raycaster" plugin="mujoco.sensor.ray_caster" 
            objtype="camera" objname="depth_camera">
      <config key="resolution" value="0.1"/>        <!-- 角分辨率(弧度) -->
      <config key="size" value="1.57 0.785"/>       <!-- FOV [水平, 垂直](弧度) -->
      <config key="dis_range" value="0.1 10"/>      <!-- 测距范围 [最小, 最大](米) -->
      <config key="type" value="world"/>            <!-- 坐标系: base/yaw/world -->
      <config key="sensor_data_types" value="pos_w"/>  <!-- 输出世界坐标点 -->
    </plugin>
    
    <!-- 可选：用于TF发布 -->
    <framepos name="depth_camera_pos" objtype="camera" objname="depth_camera"/>
    <framequat name="depth_camera_quat" objtype="camera" objname="depth_camera"/>
  </sensor>
</mujoco>
```

## RViz2可视化

1. 启动RViz2: `rviz2`
2. 设置Fixed Frame为 `world`
3. 添加PointCloud2显示：Add -> By topic -> `/raycaster/<sensor_name>` -> PointCloud2
4. 调整点的大小和颜色

## 与主仿真程序配合使用

可以同时运行主仿真程序和ROS2节点：

**终端1** - 主仿真（机器人控制）:
```bash
cd ~/clone/unitree_mujoco/simulate/build
./unitree_mujoco
```

**终端2** - ROS2节点（传感器数据发布）:
```bash
cd ~/clone/unitree_mujoco/simulate/raycaster_ros2
source ros2_ws/install/setup.bash
ros2 run raycaster_ros2 raycaster_ros2_node --ros-args \
  -p model_file:=~/clone/unitree_mujoco/unitree_robots/go2/scene.xml
```

这样实现了控制和传感器数据发布的完全解耦。

## 故障排查

### 编译错误

```bash
# 确保已source ROS2环境
source /opt/ros/humble/setup.bash

# 清理后重新编译
cd ~/clone/unitree_mujoco/simulate/raycaster_ros2
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
./build.sh
```

### 找不到libmujoco.so

LD_LIBRARY_PATH已在脚本中设置，如果手动运行节点需要：
```bash
export LD_LIBRARY_PATH=~/clone/unitree_mujoco/simulate/mujoco/lib:$LD_LIBRARY_PATH
```

### 找不到raycaster传感器

检查：
- 模型中是否正确配置了raycaster插件
- plugin的type参数是否设置（必需：base/yaw/world）
- 查看节点启动日志中的传感器列表

### 点云数据为空

检查：
- 场景中是否有可检测的几何体
- dis_range范围是否合理
- 使用测试模型验证：`./run_simple.sh`

## 文件结构

```
raycaster_ros2/
├── raycaster_ros2_node.h        # 节点头文件
├── raycaster_ros2_node.cpp      # 节点实现
├── CMakeLists.txt               # CMake配置
├── package.xml                  # ROS2包定义
├── build.sh                     # 一键编译脚本
├── run_simple.sh                # 快速测试脚本
├── run_test.sh                  # 使用launch文件运行
├── launch/
│   └── raycaster.launch.py      # ROS2启动文件
├── models/
│   └── raycaster_test.xml       # 测试模型
├── mujoco_plugin/               # 插件库目录（编译后生成）
└── ros2_ws/                     # ROS2工作空间（编译后生成）
```

## 技术说明

节点在初始化时会：
1. 加载MuJoCo模型
2. 加载raycaster插件
3. 运行几步仿真以初始化插件
4. 自动发现所有raycaster传感器
5. 为每个传感器创建ROS2 publisher
6. 启动仿真和发布线程

数据发布流程：
1. 在独立线程中运行MuJoCo仿真(mj_step)
2. 从plugin_state读取传感器配置（ray数量、数据偏移）
3. 从sensordata读取点云数据
4. 转换为PointCloud2消息并发布

## 开发说明

如需修改：
- 修改代码后运行`./build.sh`重新编译
- 修改launch文件后无需重新编译
- 修改模型文件后直接使用新模型路径运行即可

## 参考

- 上级文档: [../RAYCASTER.md](../RAYCASTER.md)
- 原始项目: https://github.com/Albusgive/mujoco_ray_caster


