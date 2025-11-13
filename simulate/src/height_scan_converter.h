#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <memory>
#include <string>
#include <vector>

class HeightScanConverter {
public:
    HeightScanConverter(rclcpp::Node::SharedPtr node, bool flatten_xyz = true, bool zero_mean = false) 
        : node_(node), flatten_xyz_(flatten_xyz), zero_mean_(zero_mean), initialized_(false) {
        
        // Default topic names
        input_topic_ = "/height_scan";
        output_topic_ = "/height_scan_array";
        
        RCLCPP_INFO(node_->get_logger(), "HeightScanConverter initialized");
    }
    
    ~HeightScanConverter() = default;
    
    // Initialize the converter with custom topic names if needed
    void initialize(const std::string& input_topic = "", const std::string& output_topic = "") {
        if (!input_topic.empty()) {
            input_topic_ = input_topic;
        }
        if (!output_topic.empty()) {
            output_topic_ = output_topic;
        }
        
        // Create subscriber and publisher
        sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, 
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointCloudCallback(msg);
            });
        
        pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            output_topic_, 10);
        
        RCLCPP_INFO(node_->get_logger(), "HeightScanConverter started");
        RCLCPP_INFO(node_->get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Flatten XYZ: %s", flatten_xyz_ ? "true" : "false");
        
        initialized_ = true;
    }
    
    // Set whether to flatten xyz coordinates or just use z values
    void setFlattenXYZ(bool flatten) {
        flatten_xyz_ = flatten;
    }
    
    // Set whether to subtract mean from z values
    void setZeroMean(bool zero_mean) {
        zero_mean_ = zero_mean;
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!initialized_) return;
        
        auto array_msg = std_msgs::msg::Float32MultiArray();
        
        // Get point cloud iterators
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        size_t num_points = msg->width * msg->height;
        
        if (flatten_xyz_) {
            // Flatten all x, y, z values into a single array: [x0, y0, z0, x1, y1, z1, ...]
            array_msg.data.reserve(num_points * 3);
            
            // First pass: collect data and optionally compute mean of z values
            float z_sum = 0.0f;
            size_t z_count = 0;
            std::vector<float> temp_data;
            temp_data.reserve(num_points * 3);
            
            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
                if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                    temp_data.push_back(*iter_x);
                    temp_data.push_back(*iter_y);
                    temp_data.push_back(*iter_z);
                    if (zero_mean_) {
                        z_sum += *iter_z;
                        z_count++;
                    }
                }
            }
            
            // Second pass: subtract mean from z values if enabled
            if (zero_mean_ && z_count > 0) {
                float z_mean = z_sum / z_count;
                for (size_t i = 2; i < temp_data.size(); i += 3) {
                    temp_data[i] -= z_mean;
                }
            }
            
            array_msg.data = std::move(temp_data);
        } else {
            // Only use z values: [z0, z1, z2, ...]
            array_msg.data.reserve(num_points);
            
            // First pass: collect z values
            for (size_t i = 0; i < num_points; ++i, ++iter_z) {
                if (std::isfinite(*iter_z)) {
                    array_msg.data.push_back(*iter_z);
                }
            }
            
            // Subtract mean if enabled
            if (zero_mean_ && !array_msg.data.empty()) {
                float z_sum = 0.0f;
                for (float z : array_msg.data) {
                    z_sum += z;
                }
                float z_mean = z_sum / array_msg.data.size();
                for (float& z : array_msg.data) {
                    z -= z_mean;
                }
            }
        }
        
        pub_->publish(array_msg);
    }
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    
    std::string input_topic_;
    std::string output_topic_;
    bool flatten_xyz_;
    bool zero_mean_;
    bool initialized_;
};
