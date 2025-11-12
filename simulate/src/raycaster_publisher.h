#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#include <string>
#include <vector>
#include <memory>
#include <map>

class RaycasterPublisher {
public:
    RaycasterPublisher(rclcpp::Node::SharedPtr node) 
        : node_(node), publish_tf_(true), initialized_(false) {
        
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
                
                // Create publisher
                // Use /height_scan as default topic for compatibility with heightmap subscriber
                std::string topic_name = "/height_scan";
                sensor.publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_name, 10);
                
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
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    };
    
    struct CameraSensor {
        std::string name;
        int quat_sensor_id;
        int pos_sensor_id;
        int quat_adr;
        int pos_adr;
        std::string frame_id;
    };
    
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
            auto msg = sensor_msgs::msg::PointCloud2();
            
            // Set header
            msg.header.stamp = node_->now();
            msg.header.frame_id = sensor.frame_id;
            
            int num_points = sensor.h_ray_num * sensor.v_ray_num;
            
            msg.height = 1;
            msg.width = num_points;
            msg.point_step = 12;
            msg.row_step = msg.point_step * msg.width;
            msg.is_dense = false;
            
            // Resize data buffer first
            msg.data.resize(msg.row_step);
            
            // Set fields after buffer resize to avoid reallocation
            msg.fields.resize(3);
            
            msg.fields[0].name = "x";
            msg.fields[0].offset = 0;
            msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[0].count = 1;
            
            msg.fields[1].name = "y";
            msg.fields[1].offset = 4;
            msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[1].count = 1;
            
            msg.fields[2].name = "z";
            msg.fields[2].offset = 8;
            msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[2].count = 1;
            
            // Get data pointer
            mjtNum* data_ptr = nullptr;
            if (sensor.pos_w_data_size > 0 && sensor.pos_w_data_offset >= 0) {
                data_ptr = d->sensordata + sensor.data_adr + sensor.pos_w_data_offset;
            } else {
                data_ptr = d->sensordata + sensor.data_adr;
            }
            
            // Copy data and filter invalid points
            float* msg_data_ptr = reinterpret_cast<float*>(msg.data.data());
            int valid_points = 0;
            for (int i = 0; i < num_points; i++) {
                float x = static_cast<float>(data_ptr[i * 3 + 0]);
                float y = static_cast<float>(data_ptr[i * 3 + 1]);
                float z = static_cast<float>(data_ptr[i * 3 + 2]);
                
                // Check if point is valid (not NaN and not at extreme distance)
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                    msg_data_ptr[valid_points * 3 + 0] = x;
                    msg_data_ptr[valid_points * 3 + 1] = y;
                    msg_data_ptr[valid_points * 3 + 2] = z;
                    valid_points++;
                }
            }
            
            // Update message size to reflect only valid points
            msg.width = valid_points;
            msg.row_step = msg.point_step * valid_points;
            msg.data.resize(msg.row_step);
            
            sensor.publisher->publish(msg);
        }
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
    
    rclcpp::Node::SharedPtr node_;
    bool publish_tf_;
    bool initialized_;
    
    std::vector<RayCasterSensor> raycaster_sensors_;
    std::map<std::string, CameraSensor> camera_sensors_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
