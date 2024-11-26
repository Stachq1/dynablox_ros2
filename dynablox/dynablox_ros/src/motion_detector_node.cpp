#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

#include "dynablox_ros/motion_detector.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // TODO: Read the --alsologtostderr flag from the command line
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Create the single node instance
  auto nh = std::make_shared<rclcpp::Node>("motion_detector_node");

  // Create the MotionDetector object
  auto tsdf_server = std::make_shared<dynablox::MotionDetector>(nh);

  // Spin the node
  rclcpp::spin(nh);  // Use spin() to keep the node running

  rclcpp::shutdown();
  return 0;
}
