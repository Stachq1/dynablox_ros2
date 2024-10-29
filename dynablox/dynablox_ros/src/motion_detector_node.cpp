#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

#include "dynablox_ros/motion_detector.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  auto nh = std::make_shared<rclcpp::Node>("motion_detector");
  auto motion_detector = std::make_shared<dynablox::MotionDetector>(nh);

  rclcpp::spin(cloud_visualizer);
  rclcpp::shutdown();

  rclcpp::spin();
  return 0;
}
