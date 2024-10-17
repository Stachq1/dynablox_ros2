cmake_minimum_required(VERSION 3.17)
project(voxblox
        VERSION 1.0.0
        DESCRIPTION "Voxblox Library"
        LANGUAGES CXX
)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(minkindr REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Threads REQUIRED)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_C_FLAGS "-Werror=implicit-function-declaration")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-std=c++14 -Wall -Wextra)

# Compile protobuf files
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(PROTOBUF_COMPILE_PYTHON true)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
add_library(${PROJECT_NAME}_proto ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

# Add voxblox library
set(${PROJECT_NAME}_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
add_library(voxblox::voxblox ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the newly generated .pb.h files to include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)

ament_target_dependencies(${PROJECT_NAME}
  Boost
  Eigen3
  minkindr
  PCL
  Protobuf
  Threads
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_proto
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)

ament_export_dependencies(Boost Eigen3 minkindr PCL Protobuf Threads)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

ament_package()
