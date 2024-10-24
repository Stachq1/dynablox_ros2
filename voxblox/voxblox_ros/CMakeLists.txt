cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(minkindr REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)

# C++14 standard
add_definitions(-std=c++17 -Wall -Wextra)
add_compile_definitions(GLOG_USE_GLOG_EXPORT)

# Include directories
include_directories(${PROJECT_NAME} PUBLIC
  include
  ${voxblox_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/transformer.cc
  src/tsdf_server.cc
)

ament_target_dependencies(${PROJECT_NAME}
  minkindr
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_conversions
  visualization_msgs
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

# Add the Voxblox generated .pb.h files to include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)

################
# INSTALLATION #
################

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

ament_export_dependencies(minkindr rclcpp sensor_msgs std_srvs tf2 tf2_ros pcl_conversions visualization_msgs voxblox voxblox_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
