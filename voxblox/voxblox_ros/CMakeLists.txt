cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(interactive_markers REQUIRED)
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
  src/esdf_server.cc
  src/interactive_slider.cc
  src/simulation_server.cc
  src/intensity_server.cc
  src/transformer.cc
  src/tsdf_server.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  tf2
  cv_bridge
  pcl_conversions
  pcl_ros
  interactive_markers
  gflags
  voxblox
  voxblox_msgs
)

############
# BINARIES #
############

add_executable(voxblox_eval
  src/voxblox_eval.cc
)
target_link_libraries(voxblox_eval ${PROJECT_NAME})

add_executable(tsdf_server
  src/tsdf_server_node.cc
)
target_link_libraries(tsdf_server ${PROJECT_NAME})

add_executable(esdf_server
  src/esdf_server_node.cc
)
target_link_libraries(esdf_server ${PROJECT_NAME})

add_executable(intensity_server
  src/intensity_server_node.cc
)
target_link_libraries(intensity_server ${PROJECT_NAME})

add_executable(simulation_eval
  src/simulation_eval.cc
)
target_link_libraries(simulation_eval ${PROJECT_NAME})

add_executable(visualize_tsdf
  src/visualize_tsdf.cc
)
target_link_libraries(visualize_tsdf ${PROJECT_NAME})

#############
# INSTALLATION #
#############

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  voxblox_eval
  tsdf_server
  esdf_server
  intensity_server
  simulation_eval
  visualize_tsdf
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

#############
# EXPORT #
#############

ament_export_dependencies(rclcpp sensor_msgs tf2 cv_bridge pcl_conversions pcl_ros interactive_markers gflags voxblox voxblox_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
