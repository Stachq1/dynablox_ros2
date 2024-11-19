cmake_minimum_required(VERSION 3.5)
project(dynablox)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find ament_cmake and other necessary packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(voxblox REQUIRED)

# Add library
add_library(${PROJECT_NAME}
  src/processing/preprocessing.cpp
  src/processing/clustering.cpp
  src/processing/tracking.cpp
  src/processing/ever_free_integrator.cpp
  src/evaluation/evaluator.cpp
  src/evaluation/ground_truth_handler.cpp
  src/evaluation/io_tools.cpp
)

# Specify include directories for the target
include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${voxblox_INCLUDE_DIRS}
)

# Add the Voxblox generated .pb.h files to include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)

# Link against other libraries if necessary
ament_target_dependencies(${PROJECT_NAME}
  PCL
  pcl_conversions
  sensor_msgs
  tf2
  voxblox
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install headers if applicable
install(DIRECTORY include/
  DESTINATION include
)

# Export the package
ament_export_dependencies(PCL pcl_conversions sensor_msgs tf2 voxblox)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Include the necessary ament macros
ament_package()
