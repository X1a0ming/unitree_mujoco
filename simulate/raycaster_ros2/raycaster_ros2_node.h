#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>

class RayCasterROS2Node : public rclcpp::Node {
public:
    RayCasterROS2Node(const std::string& node_name = "raycaster_publisher")
        : Node(node_name), running_(false) {
        
        // Declare parameters
        this->declare_parameter<std::string>("model_file", "");
        this->declare_parameter<double>("publish_rate", 30.0);
        this->declare_parameter<bool>("publish_tf", true);
        
        // Get parameters
        model_file_ = this->get_parameter("model_file").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        
        if (model_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "model_file parameter is required!");
            return;
        }
        
        // Load MuJoCo model
        if (!load_model(model_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", model_file_.c_str());
            return;
        }
        
        // Initialize raycaster sensors
        initialize_raycaster_sensors();
        
        // Create TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        
        RCLCPP_INFO(this->get_logger(), "RayCaster ROS2 node initialized");
        RCLCPP_INFO(this->get_logger(), "Found %zu raycaster sensor(s)", raycaster_sensors_.size());
    }
    
    ~RayCasterROS2Node() {
        stop();
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }
    
    void start() {
        if (running_) return;
        running_ = true;
        
        // Start simulation thread
        sim_thread_ = std::thread(&RayCasterROS2Node::simulation_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "Simulation and publishing started");
    }
    
    void stop() {
        running_ = false;
        if (sim_thread_.joinable()) {
            sim_thread_.join();
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
    
    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    std::string model_file_;
    double publish_rate_;
    bool publish_tf_;
    
    std::vector<RayCasterSensor> raycaster_sensors_;
    std::map<std::string, CameraSensor> camera_sensors_;
    
    std::atomic<bool> running_;
    std::thread sim_thread_;
    std::mutex data_mutex_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    bool load_model(const std::string& filename) {
        char error[1000] = "Could not load model";
        
        // Load plugins first
        load_plugins();
        
        // Load model
        if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".mjb") {
            m_ = mj_loadModel(filename.c_str(), nullptr);
        } else {
            m_ = mj_loadXML(filename.c_str(), nullptr, error, 1000);
        }
        
        if (!m_) {
            RCLCPP_ERROR(this->get_logger(), "Load model error: %s", error);
            return false;
        }
        
        // Create data
        d_ = mj_makeData(m_);
        if (!d_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create mjData");
            mj_deleteModel(m_);
            m_ = nullptr;
            return false;
        }
        
        // Reset to initial state
        mj_resetData(m_, d_);
        mj_forward(m_, d_);
        
        return true;
    }
    
    void load_plugins() {
        // Check built-in plugins
        int nplugin = mjp_pluginCount();
        if (nplugin) {
            RCLCPP_INFO(this->get_logger(), "Built-in plugins:");
            for (int i = 0; i < nplugin; ++i) {
                RCLCPP_INFO(this->get_logger(), "  %s", mjp_getPluginAtSlot(i)->name);
            }
        }
        
        // Try to load plugins from directory
        const char* plugin_dir = "mujoco_plugin";
        mj_loadAllPluginLibraries(plugin_dir, 
            +[](const char* filename, int first, int count) {
                printf("Plugins registered by library '%s':\n", filename);
                for (int i = first; i < first + count; ++i) {
                    printf("  %s\n", mjp_getPluginAtSlot(i)->name);
                }
            });
    }
    
    void initialize_raycaster_sensors() {
        // Run a few simulation steps to ensure plugins are fully initialized
        for (int i = 0; i < 3; i++) {
            mj_step(m_, d_);
        }
        
        for (int i = 0; i < m_->nsensor; i++) {
            const char* sensor_name = mj_id2name(m_, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            int plugin_id = m_->sensor_plugin[i];
            if (plugin_id < 0) continue;
            
            const mjpPlugin* plugin = mjp_getPluginAtSlot(m_->plugin[plugin_id]);
            if (!plugin || !plugin->name) continue;
            
            std::string plugin_name(plugin->name);
            
            // Check if it's a raycaster plugin
            if (plugin_name.find("mujoco.sensor.ray_caster") != std::string::npos) {
                RayCasterSensor sensor;
                sensor.name = sensor_name;
                sensor.sensor_id = i;
                sensor.data_adr = m_->sensor_adr[i];
                sensor.plugin_id = plugin_id;
                sensor.state_adr = m_->plugin_stateadr[plugin_id];
                sensor.frame_id = "world";
                
                // Get raycaster info from plugin state
                if (d_ && sensor.state_adr >= 0 && m_->plugin_statenum[plugin_id] >= 2) {
                    sensor.h_ray_num = static_cast<int>(d_->plugin_state[sensor.state_adr + 0]);
                    sensor.v_ray_num = static_cast<int>(d_->plugin_state[sensor.state_adr + 1]);
                    
                    // Get data configuration from plugin state
                    // State layout: [h_ray, v_ray, data_offset1, data_size1, ...]
                    std::vector<std::pair<int, int>> data_configs;
                    for (int j = sensor.state_adr + 2; 
                         j < sensor.state_adr + m_->plugin_statenum[plugin_id]; 
                         j += 2) {
                        int offset = static_cast<int>(d_->plugin_state[j]);
                        int size = static_cast<int>(d_->plugin_state[j + 1]);
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
                std::string topic_name = std::string("/raycaster/") + sensor_name;
                sensor.publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_name, 10);
                
                raycaster_sensors_.push_back(std::move(sensor));
                
                RCLCPP_INFO(this->get_logger(), "RayCaster sensor: %s (topic: %s, %dx%d rays)",
                    sensor_name, topic_name.c_str(), sensor.h_ray_num, sensor.v_ray_num);
            }
        }
        
        // Also find camera sensors for TF
        if (publish_tf_) {
            find_camera_sensors();
        }
    }
    
    void find_camera_sensors() {
        // Look for camera position and quaternion sensors
        for (int i = 0; i < m_->nsensor; i++) {
            const char* sensor_name = mj_id2name(m_, mjOBJ_SENSOR, i);
            if (!sensor_name) continue;
            
            std::string name_str(sensor_name);
            
            // Check for position sensor
            if (name_str.find("_pos") != std::string::npos) {
                std::string base_name = name_str.substr(0, name_str.find("_pos"));
                
                if (camera_sensors_.find(base_name) == camera_sensors_.end()) {
                    camera_sensors_[base_name] = CameraSensor();
                    camera_sensors_[base_name].name = base_name;
                    camera_sensors_[base_name].frame_id = base_name;
                }
                
                camera_sensors_[base_name].pos_sensor_id = i;
                camera_sensors_[base_name].pos_adr = m_->sensor_adr[i];
            }
            
            // Check for quaternion sensor
            if (name_str.find("_quat") != std::string::npos) {
                std::string base_name = name_str.substr(0, name_str.find("_quat"));
                
                if (camera_sensors_.find(base_name) == camera_sensors_.end()) {
                    camera_sensors_[base_name] = CameraSensor();
                    camera_sensors_[base_name].name = base_name;
                    camera_sensors_[base_name].frame_id = base_name;
                }
                
                camera_sensors_[base_name].quat_sensor_id = i;
                camera_sensors_[base_name].quat_adr = m_->sensor_adr[i];
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Found %zu camera sensor(s) for TF", camera_sensors_.size());
    }
    
    void simulation_loop() {
        auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / publish_rate_));
        auto next_publish = std::chrono::steady_clock::now();
        
        while (running_ && rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                
                // Step simulation
                mj_step(m_, d_);
            }
            
            // Publish at specified rate
            auto now = std::chrono::steady_clock::now();
            if (now >= next_publish) {
                publish_raycaster_data();
                
                if (publish_tf_) {
                    publish_transforms();
                }
                
                next_publish += period;
            }
            
            // Small sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    
    void publish_raycaster_data() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        for (auto& sensor : raycaster_sensors_) {
            auto msg = sensor_msgs::msg::PointCloud2();
            
            // Set header
            msg.header.stamp = this->now();
            msg.header.frame_id = sensor.frame_id;
            
            // Set fields
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
            
            int num_points = sensor.h_ray_num * sensor.v_ray_num;
            
            msg.height = 1;
            msg.width = num_points;
            msg.point_step = 12;
            msg.row_step = msg.point_step * msg.width;
            msg.is_dense = false;
            
            msg.data.resize(msg.row_step);
            
            // Get data pointer
            mjtNum* data_ptr = nullptr;
            if (sensor.pos_w_data_size > 0 && sensor.pos_w_data_offset >= 0) {
                data_ptr = d_->sensordata + sensor.data_adr + sensor.pos_w_data_offset;
            } else {
                data_ptr = d_->sensordata + sensor.data_adr;
            }
            
            // Copy data
            float* msg_data_ptr = reinterpret_cast<float*>(msg.data.data());
            for (int i = 0; i < num_points; i++) {
                msg_data_ptr[i * 3 + 0] = static_cast<float>(data_ptr[i * 3 + 0]);
                msg_data_ptr[i * 3 + 1] = static_cast<float>(data_ptr[i * 3 + 1]);
                msg_data_ptr[i * 3 + 2] = static_cast<float>(data_ptr[i * 3 + 2]);
            }
            
            sensor.publisher->publish(msg);
        }
    }
    
    void publish_transforms() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        auto now = this->now();
        
        for (const auto& [name, camera] : camera_sensors_) {
            if (camera.pos_adr < 0 || camera.quat_adr < 0) continue;
            
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "world";
            t.child_frame_id = camera.frame_id;
            
            // Position
            t.transform.translation.x = d_->sensordata[camera.pos_adr + 0];
            t.transform.translation.y = d_->sensordata[camera.pos_adr + 1];
            t.transform.translation.z = d_->sensordata[camera.pos_adr + 2];
            
            // Quaternion
            t.transform.rotation.w = d_->sensordata[camera.quat_adr + 0];
            t.transform.rotation.x = d_->sensordata[camera.quat_adr + 1];
            t.transform.rotation.y = d_->sensordata[camera.quat_adr + 2];
            t.transform.rotation.z = d_->sensordata[camera.quat_adr + 3];
            
            tf_broadcaster_->sendTransform(t);
        }
    }
};
