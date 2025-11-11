cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

# Option to enable ROS2 support
option(ENABLE_ROS2 "Enable ROS2 raycaster publishing" OFF)

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
  message(STATUS "ROS2 support enabled")
  add_definitions(-DENABLE_ROS2)
endif()


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
  raycaster_src
  raycaster_plugin
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
file(GLOB RAYCASTER_SRC raycaster_src/*.cpp raycaster_src/*.h raycaster_src/*.hpp)
file(GLOB RAYCASTER_PLUGIN_SRC raycaster_plugin/*.cc raycaster_plugin/*.h)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

# Build raycaster plugin as shared library
add_library(sensor_ray SHARED ${RAYCASTER_PLUGIN_SRC} ${RAYCASTER_SRC})
target_include_directories(sensor_ray PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} raycaster_src raycaster_plugin mujoco/include)
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
if(ENABLE_ROS2)
  ament_target_dependencies(unitree_mujoco
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
  )
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)