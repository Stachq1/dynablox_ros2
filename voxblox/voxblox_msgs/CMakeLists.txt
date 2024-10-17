cmake_minimum_required(VERSION 3.5)
project(voxblox_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Specify the message files
set(msg_files
  "msg/Block.msg"
  "msg/Layer.msg"
  "msg/Mesh.msg"
  "msg/MeshBlock.msg"
  "msg/VoxelEvaluationDetails.msg"
)

# Specify the service files
set(srv_files
  "srv/FilePath.srv"
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Generate package
ament_package()