#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  auto nh = std::make_shared<rclcpp::Node>("tsdf");
  auto nh_private = std::make_shared<rclcpp::Node>("tsdf_private");
  auto tsdf_server = std::make_shared<voxblox::TsdfServer>(nh, nh_private);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
