#ifndef DYNABLOX_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_
#define DYNABLOX_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_

#include <string>
#include <vector>

#include "dynablox/common/types.h"
#include "dynablox_ros/visualization/motion_visualizer.h"

namespace dynablox {

class CloudVisualizer {
 public:
  // Config.
  struct Config {
    // File to load cloud data from.
    std::string file_path;

    // How frequently visualizations should be republished [s].
    float refresh_rate = 0.25;
  };

  // Setup.
  CloudVisualizer(rclcpp::Node::SharedPtr nh);
  void readClouds();
  void visualizeClouds();

 private:
  const Config config_;
  MotionVisualizer visualizer_;

  // ROS.
  rclcpp::Node::SharedPtr nh_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data to visualize.
  std::vector<Cloud> clouds_;
  std::vector<CloudInfo> cloud_infos_;
  std::vector<Clusters> clusters_;
};

}  // namespace dynablox

#endif  // DYNABLOX_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_
