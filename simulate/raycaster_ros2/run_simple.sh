#!/bin/bash

# 简单运行脚本 - 直接运行节点（不使用launch）

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$SCRIPT_DIR/ros2_ws"
SIMULATE_DIR="$(dirname "$SCRIPT_DIR")"

# 检查是否已构建
if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo "Error: Workspace not built. Please run ./build.sh first"
    exit 1
fi

# 设置库路径
export LD_LIBRARY_PATH="$SIMULATE_DIR/mujoco/lib:$LD_LIBRARY_PATH"

# Source工作空间
source "$WORKSPACE_DIR/install/setup.bash"

# 使用测试模型
MODEL_FILE="$SCRIPT_DIR/models/raycaster_test.xml"

echo "========================================="
echo "RayCaster ROS2 Node - Direct Run"
echo "========================================="
echo "Model: $MODEL_FILE"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# 直接运行节点
ros2 run raycaster_ros2 raycaster_ros2_node --ros-args \
    -p model_file:="$MODEL_FILE" \
    -p publish_rate:=30.0 \
    -p publish_tf:=true
