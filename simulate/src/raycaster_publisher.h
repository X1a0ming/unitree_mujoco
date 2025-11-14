#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#include "param.h"

#include <string>
#include <vector>
#include <memory>
#include <map>

class RaycasterPublisher {
public:
    enum class OutputFormat {
        POINTCLOUD,
        ARRAY
    };
    
    RaycasterPublisher(rclcpp::Node::SharedPtr node, OutputFormat format = OutputFormat::POINTCLOUD, 
                       bool flatten_xyz = true, bool zero_mean = false) 
        : node_(node), publish_tf_(true), initialized_(false), 
          output_format_(format), flatten_xyz_(flatten_xyz), zero_mean_(zero_mean) {
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    }
    
    ~RaycasterPublisher() = default;
    
    // Initialize raycaster sensors from the model
    void initialize(mjModel* m, mjData* d) {
        if (!m || !d) return;
        
        // Run a few simulation steps to ensure plugins are fully initialized
        for (int i = 0; i < 3; i++) {
            mj_step(m, d);
        }
        
        for (int i = 0; i < m->nsensor; i++) {
            const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            int plugin_id = m->sensor_plugin[i];
            if (plugin_id < 0) continue;
            
            const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[plugin_id]);
            if (!plugin || !plugin->name) continue;
            
            std::string plugin_name(plugin->name);
            
            // Check if it's a raycaster plugin
            if (plugin_name.find("mujoco.sensor.ray_caster") != std::string::npos) {
                RayCasterSensor sensor;
                sensor.name = sensor_name;
                sensor.sensor_id = i;
                sensor.data_adr = m->sensor_adr[i];
                sensor.plugin_id = plugin_id;
                sensor.state_adr = m->plugin_stateadr[plugin_id];
                sensor.frame_id = "world";
                
                // Get raycaster info from plugin state
                if (d && sensor.state_adr >= 0 && m->plugin_statenum[plugin_id] >= 2) {
                    sensor.h_ray_num = static_cast<int>(d->plugin_state[sensor.state_adr + 0]);
                    sensor.v_ray_num = static_cast<int>(d->plugin_state[sensor.state_adr + 1]);
                    
                    // Get data configuration from plugin state
                    std::vector<std::pair<int, int>> data_configs;
                    for (int j = sensor.state_adr + 2; 
                         j < sensor.state_adr + m->plugin_statenum[plugin_id]; 
                         j += 2) {
                        int offset = static_cast<int>(d->plugin_state[j]);
                        int size = static_cast<int>(d->plugin_state[j + 1]);
                        data_configs.push_back({offset, size});
                    }
                    
                    // Use the first data configuration (usually pos_w)
                    if (!data_configs.empty()) {
                        sensor.pos_w_data_offset = data_configs[0].first;
                        sensor.pos_w_data_size = data_configs[0].second;
                    } else {
                        sensor.pos_w_data_offset = 0;
                        sensor.pos_w_data_size = sensor.h_ray_num * sensor.v_ray_num * 3;
                    }
                }
                
                // Create publisher using sensor name as topic
                std::string topic_name = "/" + std::string(sensor_name);
                
                // Get sensor-specific configuration or use defaults
                std::string output_format_str = param::config.raycaster_output_format;
                bool flatten_xyz = param::config.raycaster_flatten_xyz;
                bool zero_mean = param::config.raycaster_zero_mean;
                float offset = 0.0f;
                std::string replace_nan = "";
                float max_distance = 0.0f;
                
                // Check if sensor has specific configuration
                auto it = param::config.raycaster_sensors.find(sensor_name);
                if (it != param::config.raycaster_sensors.end()) {
                    if (!it->second.output_format.empty()) {
                        output_format_str = it->second.output_format;
                    }
                    flatten_xyz = it->second.flatten_xyz;
                    zero_mean = it->second.zero_mean;
                    offset = it->second.offset;
                    replace_nan = it->second.replace_nan;
                    max_distance = it->second.max_distance;
                }
                
                // Determine output format for this sensor
                OutputFormat sensor_format = (output_format_str == "array") ? 
                    OutputFormat::ARRAY : OutputFormat::POINTCLOUD;
                
                sensor.output_format = sensor_format;
                sensor.flatten_xyz = flatten_xyz;
                sensor.zero_mean = zero_mean;
                sensor.offset = offset;
                sensor.replace_nan = replace_nan;
                sensor.max_distance = max_distance;
                
                if (sensor_format == OutputFormat::POINTCLOUD) {
                    sensor.pc_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_name, 10);
                    
                    // Pre-allocate message template to avoid repeated allocation
                    int num_points = sensor.h_ray_num * sensor.v_ray_num;
                    sensor.pc_msg_template.header.frame_id = sensor_name;
                    sensor.pc_msg_template.height = 1;
                    sensor.pc_msg_template.point_step = 12;
                    sensor.pc_msg_template.is_dense = false;
                    
                    sensor.pc_msg_template.fields.resize(3);
                    sensor.pc_msg_template.fields[0].name = "x";
                    sensor.pc_msg_template.fields[0].offset = 0;
                    sensor.pc_msg_template.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    sensor.pc_msg_template.fields[0].count = 1;
                    
                    sensor.pc_msg_template.fields[1].name = "y";
                    sensor.pc_msg_template.fields[1].offset = 4;
                    sensor.pc_msg_template.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    sensor.pc_msg_template.fields[1].count = 1;
                    
                    sensor.pc_msg_template.fields[2].name = "z";
                    sensor.pc_msg_template.fields[2].offset = 8;
                    sensor.pc_msg_template.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    sensor.pc_msg_template.fields[2].count = 1;
                    
                    sensor.pc_msg_template.data.resize(num_points * 12);
                } else {
                    // Array format
                    topic_name += "_array";
                    sensor.array_publisher = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
                        topic_name, 10);
                    
                    int num_points = sensor.h_ray_num * sensor.v_ray_num;
                    if (flatten_xyz_) {
                        sensor.array_msg_template.data.reserve(num_points * 3);
                    } else {
                        sensor.array_msg_template.data.reserve(num_points);
                    }
                }
                
                raycaster_sensors_.push_back(std::move(sensor));
                
                RCLCPP_INFO(node_->get_logger(), "RayCaster sensor: %s (topic: %s, %dx%d rays)",
                    sensor_name, topic_name.c_str(), sensor.h_ray_num, sensor.v_ray_num);
            }
        }
        
        // Find camera sensors for TF
        if (publish_tf_) {
            find_camera_sensors(m);
        }
        
        RCLCPP_INFO(node_->get_logger(), "Raycaster publisher initialized with %zu sensor(s)", 
                    raycaster_sensors_.size());
        
        initialized_ = true;
    }
    
    // Publish raycaster data and TF
    void publish(mjModel* m, mjData* d) {
        if (!m || !d || !initialized_) return;
        
        publish_raycaster_data(d);
        
        if (publish_tf_) {
            publish_transforms(d);
        }
    }

private:
    struct RayCasterSensor {
        std::string name;
        int sensor_id;
        int data_adr;
        int plugin_id;
        int state_adr;
        int h_ray_num;
        int v_ray_num;
        int pos_w_data_offset;
        int pos_w_data_size;
        std::string frame_id;
        
        // Per-sensor configuration
        OutputFormat output_format;
        bool flatten_xyz;
        bool zero_mean;
        float offset;              // Offset to apply to distances
        std::string replace_nan;  // "zero", "max", or ""
        float max_distance;        // Maximum distance from sensor config
        
        // Publishers for different formats
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr array_publisher;
        
        // Pre-allocated buffers
        sensor_msgs::msg::PointCloud2 pc_msg_template;
        std_msgs::msg::Float32MultiArray array_msg_template;
    };
    
    struct CameraSensor {
        std::string name;
        int quat_sensor_id;
        int pos_sensor_id;
        int quat_adr;
        int pos_adr;
        std::string frame_id;
    };
    
    // Member variables
    rclcpp::Node::SharedPtr node_;
    bool publish_tf_;
    bool initialized_;
    OutputFormat output_format_;
    bool flatten_xyz_;
    bool zero_mean_;
    
    std::vector<RayCasterSensor> raycaster_sensors_;
    std::map<std::string, CameraSensor> camera_sensors_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Private methods
    void find_camera_sensors(mjModel* m) {
        for (int i = 0; i < m->nsensor; i++) {
            const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            std::string name_str(sensor_name);
            
            // Check for position sensor
            if (name_str.find("_pos") != std::string::npos) {
                std::string base_name = name_str.substr(0, name_str.find("_pos"));
                
                if (camera_sensors_.find(base_name) == camera_sensors_.end()) {
                    camera_sensors_[base_name] = CameraSensor();
                    camera_sensors_[base_name].name = base_name;
                    camera_sensors_[base_name].frame_id = base_name;
                    camera_sensors_[base_name].quat_sensor_id = -1;
                    camera_sensors_[base_name].pos_sensor_id = -1;
                    camera_sensors_[base_name].quat_adr = -1;
                    camera_sensors_[base_name].pos_adr = -1;
                }
                
                camera_sensors_[base_name].pos_sensor_id = i;
                camera_sensors_[base_name].pos_adr = m->sensor_adr[i];
            }
            
            // Check for quaternion sensor
            if (name_str.find("_quat") != std::string::npos) {
                std::string base_name = name_str.substr(0, name_str.find("_quat"));
                
                if (camera_sensors_.find(base_name) == camera_sensors_.end()) {
                    camera_sensors_[base_name] = CameraSensor();
                    camera_sensors_[base_name].name = base_name;
                    camera_sensors_[base_name].frame_id = base_name;
                    camera_sensors_[base_name].quat_sensor_id = -1;
                    camera_sensors_[base_name].pos_sensor_id = -1;
                    camera_sensors_[base_name].quat_adr = -1;
                    camera_sensors_[base_name].pos_adr = -1;
                }
                
                camera_sensors_[base_name].quat_sensor_id = i;
                camera_sensors_[base_name].quat_adr = m->sensor_adr[i];
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), "Found %zu camera sensor(s) for TF", camera_sensors_.size());
    }
    
    void publish_raycaster_data(mjData* d) {
        for (auto& sensor : raycaster_sensors_) {
            if (sensor.output_format == OutputFormat::POINTCLOUD) {
                publish_sensor_as_pointcloud(sensor, d);
            } else {
                publish_sensor_as_array(sensor, d);
            }
        }
    }
    
    void publish_sensor_as_pointcloud(RayCasterSensor& sensor, mjData* d) {
        auto& msg = sensor.pc_msg_template;
        msg.header.stamp = node_->now();
            
            int num_points = sensor.h_ray_num * sensor.v_ray_num;
            
            // Get data pointer
            mjtNum* data_ptr = nullptr;
            if (sensor.pos_w_data_size > 0 && sensor.pos_w_data_offset >= 0) {
                data_ptr = d->sensordata + sensor.data_adr + sensor.pos_w_data_offset;
            } else {
                data_ptr = d->sensordata + sensor.data_adr;
            }
            
            // Fast copy with type conversion and offset application
            float* msg_data_ptr = reinterpret_cast<float*>(msg.data.data());
            for (int i = 0; i < num_points; i++) {
                msg_data_ptr[i * 3 + 0] = static_cast<float>(data_ptr[i * 3 + 0]);
                msg_data_ptr[i * 3 + 1] = static_cast<float>(data_ptr[i * 3 + 1]);
                msg_data_ptr[i * 3 + 2] = static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset;
            }

            msg.width = num_points;
            msg.row_step = msg.point_step * num_points;

            sensor.pc_publisher->publish(msg);
    }

    void publish_sensor_as_array(RayCasterSensor& sensor, mjData* d) {
        auto& msg = sensor.array_msg_template;
        msg.data.clear();

        int num_points = sensor.h_ray_num * sensor.v_ray_num;

        // Get data pointer
        mjtNum* data_ptr = nullptr;
        if (sensor.pos_w_data_size > 0 && sensor.pos_w_data_offset >= 0) {
            data_ptr = d->sensordata + sensor.data_adr + sensor.pos_w_data_offset;
        } else {
            data_ptr = d->sensordata + sensor.data_adr;
        }

        // Helper function to replace non-finite values
        auto replace_nonfinite = [&](float value) -> float {
            if (!std::isfinite(value)) {
                if (sensor.replace_nan == "zero") {
                    return 0.0f;
                } else if (sensor.replace_nan == "max") {
                    return sensor.max_distance;
                }
            }
            return value;
        };

        if (sensor.flatten_xyz) {
            // Flatten all x, y, z values: [x0, y0, z0, x1, y1, z1, ...]
            msg.data.reserve(num_points * 3);

            if (sensor.zero_mean) {
                // First pass: compute mean of z values (after offset)
                float z_sum = 0.0f;
                int z_count = 0;
                for (int i = 0; i < num_points; i++) {
                    float z = replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset);
                    if (std::isfinite(z)) {
                        z_sum += z;
                        z_count++;
                    }
                }
                float z_mean = (z_count > 0) ? (z_sum / z_count) : 0.0f;

                // Second pass: subtract mean from z
                for (int i = 0; i < num_points; i++) {
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 0])));
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 1])));
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset) - z_mean);
                }
            } else {
                // No mean subtraction, but apply offset
                for (int i = 0; i < num_points; i++) {
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 0])));
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 1])));
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset));
                }
            }
        } else {
            // Only z values: [z0, z1, z2, ...]
            msg.data.reserve(num_points);

            if (sensor.zero_mean) {
                // Compute mean (after offset)
                float z_sum = 0.0f;
                int z_count = 0;
                for (int i = 0; i < num_points; i++) {
                    float z = replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset);
                    if (std::isfinite(z)) {
                        z_sum += z;
                        z_count++;
                    }
                }
                float z_mean = (z_count > 0) ? (z_sum / z_count) : 0.0f;

                // Subtract mean
                for (int i = 0; i < num_points; i++) {
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset) - z_mean);
                }
            } else {
                // No mean subtraction, but apply offset
                for (int i = 0; i < num_points; i++) {
                    msg.data.push_back(replace_nonfinite(static_cast<float>(data_ptr[i * 3 + 2]) + sensor.offset));
                }
            }
        }
        
        sensor.array_publisher->publish(msg);
    }
    
    void publish_transforms(mjData* d) {
        auto now = node_->now();
        
        for (const auto& [name, camera] : camera_sensors_) {
            if (camera.pos_adr < 0 || camera.quat_adr < 0) continue;
            
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "world";
            t.child_frame_id = camera.frame_id;
            
            // Position
            t.transform.translation.x = d->sensordata[camera.pos_adr + 0];
            t.transform.translation.y = d->sensordata[camera.pos_adr + 1];
            t.transform.translation.z = d->sensordata[camera.pos_adr + 2];
            
            // Quaternion
            t.transform.rotation.w = d->sensordata[camera.quat_adr + 0];
            t.transform.rotation.x = d->sensordata[camera.quat_adr + 1];
            t.transform.rotation.y = d->sensordata[camera.quat_adr + 2];
            t.transform.rotation.z = d->sensordata[camera.quat_adr + 3];
            
            tf_broadcaster_->sendTransform(t);
        }
    }
};
