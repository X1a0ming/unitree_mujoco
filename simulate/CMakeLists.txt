cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

# Option to enable ROS2 support (includes raycaster and gridmap)
option(ENABLE_ROS2 "Enable ROS2 support (raycaster + gridmap)" OFF)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)

# Find ROS2 if enabled
if(ENABLE_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common)
  find_package(pcl_conversions REQUIRED)
  message(STATUS "ROS2 support enabled (raycaster + gridmap)")
  add_definitions(-DENABLE_ROS2)
endif()


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
  raycaster/src
  raycaster/plugin
  src
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

# Raycaster plugin source files
file(GLOB RAYCASTER_SRC raycaster/src/*.cpp raycaster/src/*.h raycaster/src/*.hpp)
file(GLOB RAYCASTER_PLUGIN_SRC raycaster/plugin/*.cc raycaster/plugin/*.h)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
)

# Build raycaster plugin as shared library FIRST (before finding grid_map to avoid pollution)
add_library(sensor_ray SHARED ${RAYCASTER_PLUGIN_SRC} ${RAYCASTER_SRC})
target_include_directories(sensor_ray PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR} 
  raycaster/src 
  raycaster/plugin 
  mujoco/include
)
target_link_libraries(sensor_ray PRIVATE mujoco Eigen3::Eigen)

# Copy plugin library to build directory for loading
add_custom_command(TARGET sensor_ray POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/mujoco_plugin"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:sensor_ray>"
        "${CMAKE_BINARY_DIR}/mujoco_plugin/"
    COMMENT "Copying sensor_ray plugin to mujoco_plugin directory..."
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco sensor_ray)

# Link ROS2 libraries if enabled
# IMPORTANT: grid_map packages must be found AFTER sensor_ray is defined
# to avoid polluting sensor_ray with EIGEN plugin definitions
if(ENABLE_ROS2)
  # Find grid_map packages here (after sensor_ray target is created)
  find_package(grid_map_msgs REQUIRED)
  find_package(grid_map_ros REQUIRED)
  find_package(grid_map_cv REQUIRED)
  find_package(grid_map_core REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(fmt REQUIRED)
  find_package(CURL REQUIRED)
  
  # Remove global definitions added by grid_map_core
  remove_definitions(-DEIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp")
  remove_definitions(-DEIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp")
  
  # Add them only to unitree_mujoco target
  target_compile_definitions(unitree_mujoco PRIVATE
    EIGEN_DENSEBASE_PLUGIN="grid_map_core/eigen_plugins/DenseBasePlugin.hpp"
    EIGEN_FUNCTORS_PLUGIN="grid_map_core/eigen_plugins/FunctorsPlugin.hpp"
  )
  
  target_include_directories(unitree_mujoco PRIVATE 
    ${PCL_INCLUDE_DIRS}
    ${grid_map_core_INCLUDE_DIRS}
    ${grid_map_ros_INCLUDE_DIRS}
    ${grid_map_cv_INCLUDE_DIRS}
  )
  target_link_libraries(unitree_mujoco 
    ${PCL_LIBRARIES}
    fmt::fmt
    CURL::libcurl
  )
  ament_target_dependencies(unitree_mujoco
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
    grid_map_msgs
    grid_map_ros
    grid_map_cv
    grid_map_core
    pcl_conversions
    nav_msgs
  )
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)