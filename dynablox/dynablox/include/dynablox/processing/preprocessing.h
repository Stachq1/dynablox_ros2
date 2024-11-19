#ifndef DYNABLOX_PROCESSING_PREPROCESSING_H_
#define DYNABLOX_PROCESSING_PREPROCESSING_H_

#include <string>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynablox/common/types.h"

namespace dynablox {

class Preprocessing {
 public:
  // Config.
  struct Config {
    // Maximum ray length to integrate [m].
    float max_range = 20.f;

    // Minimum range for all points [m].
    float min_range = 0.5;
  };

  /**
   * @brief Transform the pointcloud to world frame and mark points valid for
   * integration and evaluation.
   *
   * @param msg Input pointcloud in sensor frame.
   * @param T_M_S Transform sensor (S) to map (M).
   * @param cloud_info Cloud info to store the data of the input cloud.
   * @param cloud Cloud to store the processed input point cloud.
   * @param msg_timestamp Timestamp of the pointcloud (passed separatly to avoid using ROS)
   * @return Success.
   */
  bool processPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                         const Eigen::Matrix4f& T_M_S, Cloud& cloud,
                         CloudInfo& cloud_info, std::uint64_t msg_timestamp) const;

 private:
  // Config.
  const Config config_;
};

}  // namespace dynablox

#endif  // DYNABLOX_PROCESSING_PREPROCESSING_H_
