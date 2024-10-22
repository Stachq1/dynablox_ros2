cmake_minimum_required(VERSION 3.8)
project(voxblox_rviz_plugin)

# Set C++ standard and additional compiler flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)

# This setting causes Qt's "MOC" generation to happen automatically
set(CMAKE_AUTOMOC ON)

# Include Qt5 for the RViz plugin
set(QT_LIBRARIES Qt5::Widgets)

# Include directories
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

# Set source and header files
set(HEADER_FILES
  include/voxblox_rviz_plugin/voxblox_mesh_display.h
  include/voxblox_rviz_plugin/voxblox_mesh_visual.h
)

set(SRC_FILES
  src/voxblox_mesh_display.cc
  src/voxblox_mesh_visual.cc
)

# Add the RViz plugin library
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)

# Link against necessary libraries
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  rviz_common
  voxblox
  voxblox_msgs
)

# Install the plugin description XML
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install icons directory
install(
  DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install the shared library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
