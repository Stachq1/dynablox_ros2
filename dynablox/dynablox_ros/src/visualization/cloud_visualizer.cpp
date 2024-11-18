#include "dynablox_ros/visualization/cloud_visualizer.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dynablox/evaluation/io_tools.h"
#include "dynablox/processing/clustering.h"

namespace dynablox {

CloudVisualizer::CloudVisualizer(rclcpp::Node::SharedPtr nh) :
      visualizer_(nh, std::make_shared<TsdfLayer>(0.2, 16)),
      nh_(nh) {
  // Load the data.
  if (!loadCloudFromCsv(config_.file_path, clouds_, cloud_infos_, clusters_)) {
    LOG(FATAL) << "Failed to read clouds from '" << config_.file_path << "'.";
  }
  LOG(INFO) << "Read " << clouds_.size() << " clouds from '"
            << config_.file_path << "'.";

  // Recompute the cluster aabbs.
  for (size_t i = 0; i < clusters_.size(); ++i) {
    const Cloud& cloud = clouds_[i];
    for (Cluster& cluster : clusters_[i]) {
      Point& min = cluster.aabb.min_corner;
      Point& max = cluster.aabb.max_corner;
      min = cloud[cluster.points[0]];
      max = cloud[cluster.points[0]];
      for (size_t i = 1; i < cluster.points.size(); ++i) {
        const Point& point = cloud[cluster.points[i]];
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
      }
    }
  }

  // Visualize periodically just in case.
  auto refresh_rate = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::duration<double>(config_.refresh_rate));
  timer_ = nh_->create_wall_timer(refresh_rate, [this]() { this->visualizeClouds(); });
}

void CloudVisualizer::visualizeClouds() {
  for (size_t i = 0; i < clouds_.size(); ++i) {
    const std::string ns = "cloud_" + std::to_string(i);
    visualizer_.visualizeGroundTruth(clouds_[i], cloud_infos_[i], ns);
    visualizer_.visualizeClusters(clusters_[i], ns);
  }
}

}  // namespace dynablox
