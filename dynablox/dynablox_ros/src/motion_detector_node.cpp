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

  // Load MotionDetector parameters from the ROS parameter server
  dynablox::MotionDetector::Config config;
  nh->declare_parameter("evaluate", config.evaluate);
  nh->declare_parameter("visualize", config.visualize);
  nh->declare_parameter("verbose", config.verbose);
  nh->declare_parameter("pointcloud_topic", config.pointcloud_topic);
  nh->declare_parameter("global_frame_name", config.global_frame_name);
  nh->declare_parameter("sensor_frame_name", config.sensor_frame_name);
  nh->declare_parameter("queue_size", config.queue_size);
  nh->declare_parameter("num_threads", config.num_threads);
  nh->declare_parameter("shutdown_after", config.shutdown_after);
  nh->get_parameter("evaluate", config.evaluate);
  nh->get_parameter("visualize", config.visualize);
  nh->get_parameter("verbose", config.verbose);
  nh->get_parameter("pointcloud_topic", config.pointcloud_topic);
  nh->get_parameter("global_frame_name", config.global_frame_name);
  nh->get_parameter("sensor_frame_name", config.sensor_frame_name);
  nh->get_parameter("queue_size", config.queue_size);
  nh->get_parameter("num_threads", config.num_threads);
  nh->get_parameter("shutdown_after", config.shutdown_after);

  if(config.num_threads < 1) {
    config.num_threads = std::thread::hardware_concurrency();
  }

  // Create the MotionDetector object
  auto tsdf_server = std::make_shared<dynablox::MotionDetector>(nh, config);

  // Spin the node
  rclcpp::spin(nh);

  rclcpp::shutdown();
  return 0;
}
