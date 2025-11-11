#include "raycaster_ros2_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // Note: Modern MuJoCo doesn't require activation
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RayCasterROS2Node>("raycaster_publisher");
    
    // Start simulation
    node->start();
    
    // Spin
    rclcpp::spin(node);
    
    // Cleanup
    node->stop();
    rclcpp::shutdown();
    
    return 0;
}
