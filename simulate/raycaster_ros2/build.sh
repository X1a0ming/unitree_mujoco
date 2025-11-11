#!/bin/bash

# RayCaster ROS2 构建脚本

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SIMULATE_DIR="$(dirname "$SCRIPT_DIR")"

echo "======================================"
echo "Building RayCaster ROS2 Node"
echo "======================================"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 environment not sourced!"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "ROS2 Distribution: $ROS_DISTRO"
echo "Script dir: $SCRIPT_DIR"
echo "Simulate dir: $SIMULATE_DIR"

# 创建ROS2工作空间
WORKSPACE_DIR="$SCRIPT_DIR/ros2_ws"
mkdir -p "$WORKSPACE_DIR/src"

# 创建软链接到raycaster_ros2源码目录
if [ -L "$WORKSPACE_DIR/src/raycaster_ros2" ]; then
    rm "$WORKSPACE_DIR/src/raycaster_ros2"
fi
ln -sf "$SCRIPT_DIR" "$WORKSPACE_DIR/src/raycaster_ros2"

# 确保raycaster plugin已编译
echo ""
echo "Building raycaster plugin..."
cd "$SIMULATE_DIR"
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake ..
make sensor_ray -j$(nproc)

# 复制plugin到raycaster_ros2目录（供独立运行使用）
mkdir -p "$SCRIPT_DIR/mujoco_plugin"
if [ -f "libsensor_ray.so" ]; then
    cp libsensor_ray.so "$SCRIPT_DIR/mujoco_plugin/" 
    echo "Plugin copied to $SCRIPT_DIR/mujoco_plugin/"
fi

echo ""
echo "Building ROS2 workspace..."
cd "$WORKSPACE_DIR"
colcon build --packages-select raycaster_ros2

echo ""
echo "======================================"
echo "Build completed successfully!"
echo "======================================"
echo ""
echo "To use the node, run:"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo "  ros2 launch raycaster_ros2 raycaster.launch.py model_file:=/path/to/model.xml"
echo ""
echo "Or directly:"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo "  ros2 run raycaster_ros2 raycaster_ros2_node --ros-args -p model_file:=/path/to/model.xml"
echo ""
