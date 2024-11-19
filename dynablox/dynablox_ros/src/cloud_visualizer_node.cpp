#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

#include "dynablox_ros/visualization/cloud_visualizer.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  auto nh = std::make_shared<rclcpp::Node>("cloud_visualizer");
  auto cloud_visualizer = std::make_shared<dynablox::CloudVisualizer>(nh);

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
