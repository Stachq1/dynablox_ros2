cmake_minimum_required(VERSION 3.5)
project(minkindr)

# Find ament
find_package(ament_cmake REQUIRED)

# Find other dependencies
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(Threads REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf_conversions REQUIRED)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(geometry_msgs tf2 tf_conversions)
ament_export_include_directories(include)
ament_package()

