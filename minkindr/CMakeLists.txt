cmake_minimum_required(VERSION 3.5)
project(minkindr)

# Find ament
find_package(ament_cmake REQUIRED)

# Find other dependencies
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Threads REQUIRED)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()

