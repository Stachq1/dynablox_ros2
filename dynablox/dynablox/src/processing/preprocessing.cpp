#include "dynablox/processing/preprocessing.h"

#include <vector>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dynablox {

void Preprocessing::Config::checkParams() const {
  checkParamGT(min_range, 0.f, "min_range");
  checkParamCond(max_range > min_range,
                 "'max_range' must be larger than 'min_range'.");
}

void Preprocessing::Config::setupParamsAndPrinting() {
  setupParam("min_range", &min_range, "m");
  setupParam("max_range", &max_range, "m");
}

Preprocessing::Preprocessing(const Config& config)
    : config_(config.checkValid()) {}

bool Preprocessing::processPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                      const Eigen::Matrix4f& T_M_S,
                                      Cloud& cloud,
                                      CloudInfo& cloud_info,
                                      std::uint64_t msg_timestamp) const {
  // Convert to ROS msg to pcl cloud.
  pcl::fromROSMsg(*msg, cloud);

  // Populate the cloud information with data for all points.
  cloud_info.timestamp = msg_timestamp;
  cloud_info.sensor_position.x = T_M_S(0, 3);
  cloud_info.sensor_position.y = T_M_S(1, 3);
  cloud_info.sensor_position.z = T_M_S(2, 3);

  cloud_info.points = std::vector<PointInfo>(cloud.size());
  size_t i = 0;
  for (const auto& point : cloud) {
    const float norm =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    PointInfo& info = cloud_info.points.at(i);
    info.distance_to_sensor = norm;
    i++;
  }

  // Transform the cloud to world frame.
  pcl::transformPointCloud(cloud, cloud, T_M_S); // Worry about this later maybe?
  return true;
}

}  // namespace dynablox
