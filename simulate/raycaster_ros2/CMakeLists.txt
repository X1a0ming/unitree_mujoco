cmake_minimum_required(VERSION 3.16)
project(raycaster_ros2)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Determine source directory
# CMAKE_CURRENT_SOURCE_DIR points to the actual directory (follows symlinks)
get_filename_component(RAYCASTER_ROS2_DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)
get_filename_component(SIMULATE_DIR ${RAYCASTER_ROS2_DIR}/.. ABSOLUTE)

message(STATUS "RayCaster ROS2 dir: ${RAYCASTER_ROS2_DIR}")
message(STATUS "Simulate dir: ${SIMULATE_DIR}")

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SIMULATE_DIR}/mujoco/include
  ${SIMULATE_DIR}/raycaster_src
  ${SIMULATE_DIR}/raycaster_plugin
)

# Link directories
link_directories(${SIMULATE_DIR}/mujoco/lib)

# Raycaster plugin source files
file(GLOB RAYCASTER_SRC "${SIMULATE_DIR}/raycaster_src/*.cpp")
file(GLOB RAYCASTER_PLUGIN_SRC "${SIMULATE_DIR}/raycaster_plugin/*.cc")

message(STATUS "Found ${CMAKE_MATCH_COUNT} raycaster source files")
message(STATUS "RAYCASTER_SRC: ${RAYCASTER_SRC}")
message(STATUS "RAYCASTER_PLUGIN_SRC: ${RAYCASTER_PLUGIN_SRC}")

# Check if we found the source files
list(LENGTH RAYCASTER_SRC RAYCASTER_SRC_COUNT)
list(LENGTH RAYCASTER_PLUGIN_SRC RAYCASTER_PLUGIN_SRC_COUNT)

if(RAYCASTER_SRC_COUNT EQUAL 0 OR RAYCASTER_PLUGIN_SRC_COUNT EQUAL 0)
  message(WARNING "Raycaster source files not found, using pre-built library")
  # Try to use pre-built library instead
  find_library(SENSOR_RAY_LIB sensor_ray PATHS ${SIMULATE_DIR}/build ${RAYCASTER_ROS2_DIR}/mujoco_plugin NO_DEFAULT_PATH)
  if(SENSOR_RAY_LIB)
    message(STATUS "Using pre-built sensor_ray library: ${SENSOR_RAY_LIB}")
    add_library(sensor_ray SHARED IMPORTED)
    set_target_properties(sensor_ray PROPERTIES IMPORTED_LOCATION ${SENSOR_RAY_LIB})
  else()
    message(FATAL_ERROR "Cannot find raycaster source files or pre-built library. Please build the main project first.")
  endif()
else()
  # Build raycaster plugin as shared library
  message(STATUS "Building sensor_ray from source")
  add_library(sensor_ray SHARED ${RAYCASTER_PLUGIN_SRC} ${RAYCASTER_SRC})
  target_include_directories(sensor_ray PRIVATE 
    ${SIMULATE_DIR}/raycaster_src 
    ${SIMULATE_DIR}/raycaster_plugin 
    ${SIMULATE_DIR}/mujoco/include
  )
  target_link_libraries(sensor_ray mujoco Eigen3::Eigen)
endif()

# Build ROS2 node
add_executable(raycaster_ros2_node raycaster_ros2_node.cpp)
ament_target_dependencies(raycaster_ros2_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(raycaster_ros2_node
  mujoco
  sensor_ray
)

# Install targets
install(TARGETS
  raycaster_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin library
if(TARGET sensor_ray AND NOT SENSOR_RAY_LIB)
  install(TARGETS sensor_ray
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS sensor_ray
    LIBRARY DESTINATION share/${PROJECT_NAME}/mujoco_plugin
  )
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
