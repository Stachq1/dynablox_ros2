cmake_minimum_required(VERSION 3.5)
project(dynablox)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ament_cmake and other necessary packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(voxblox REQUIRED)

# Include directories
include_directories(include)

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
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${voxblox_INCLUDE_DIRS}
)

# Link against other libraries if necessary
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  pcl_conversions
  pcl_ros
  tf2
  voxblox
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)

# Install headers if applicable (you may need to adjust the path)
install(DIRECTORY include/
  DESTINATION include
)

# Export the package
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME})
ament_export_dependencies(PCL pcl_conversions pcl_ros tf2 voxblox)
ament_export_include_directories(include)

# Include the necessary ament macros
ament_package()
