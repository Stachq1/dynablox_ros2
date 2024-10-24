cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(gflags REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)

# C++14 standard
add_definitions(-std=c++14 -Wall -Wextra)
add_compile_definitions(GLOG_USE_GLOG_EXPORT)

# Include directories
include_directories(include)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/transformer.cc
  src/tsdf_server.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  tf2
  pcl_conversions
  pcl_ros
  gflags
  voxblox
  voxblox_msgs
)

############
# BINARIES #
############

add_executable(tsdf_server
  src/tsdf_server_node.cc
)
target_link_libraries(tsdf_server ${PROJECT_NAME})

#############
# INSTALLATION #
#############

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  tsdf_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

#############
# EXPORT #
#############

ament_export_dependencies(rclcpp sensor_msgs tf2 pcl_conversions pcl_ros gflags voxblox voxblox_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
