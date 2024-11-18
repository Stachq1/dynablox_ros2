#ifndef DYNABLOX_ROS_VISUALIZATION_MOTION_VISUALIZER_H_
#define DYNABLOX_ROS_VISUALIZATION_MOTION_VISUALIZER_H_

#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_ros/tsdf_server.h>

#include "dynablox/common/types.h"

namespace dynablox {

class MotionVisualizer {
 public:
  // Config.
  struct Config {
    std::string global_frame_name = "map";

    // Set RGBA colors in [0, 1] if wanted.
    std::vector<float> static_point_color = {0.f, 0.f, 0.f, 1.f};
    std::vector<float> dynamic_point_color = {1.f, 0.f, 0.5f, 1.f};
    std::vector<float> sensor_color = {1.f, 0.f, 0.f, 1.f};
    std::vector<float> true_positive_color = {0.f, 1.f, 0.f, 1.f};
    std::vector<float> false_positive_color = {1.f, 0.f, 0.f, 1.f};
    std::vector<float> true_negative_color = {0.f, 0.f, 0.f, 1.f};
    std::vector<float> false_negative_color = {0.f, 0.f, 1.f, 1.f};
    std::vector<float> out_of_bounds_color = {.7f, .7f, .7f, 1.f};
    std::vector<float> ever_free_color = {1.f, 0.f, 1.f, .5f};
    std::vector<float> never_free_color = {0.f, 1.f, 1.f, .5f};
    std::vector<float> point_level_slice_color = {1.f, 0.f, 1.f, 1.f};
    std::vector<float> cluster_level_slice_color = {0.f, 1.f, 1.f, 1.f};

    // Scales of visualizations [m].
    float static_point_scale = 0.1f;
    float dynamic_point_scale = 0.1f;
    float sensor_scale = 0.3f;
    float cluster_line_width = 0.05f;

    // Number of colors for the a full color wheel revolution.
    int color_wheel_num_colors = 20;

    // True: every cluster and object has a color, False: just mark them as
    // dynamic.
    bool color_clusters = true;

    // Height in map frame of the slice being visualized [m].
    float slice_height = 0;

    // True: slice height is relative to the sensor, False: slice in world frame
    bool slice_relative_to_sensor = true;

    // Crop all visualizations at this height for better visibility.
    float visualization_max_z = 10000.f;
  };

  // Setup.
  MotionVisualizer(rclcpp::Node::SharedPtr nh, std::shared_ptr<TsdfLayer> tsdf_layer);

  void setupRos();

  // Visualization.
  void visualizeAll(const Cloud& cloud, const CloudInfo& cloud_info,
                    const Clusters& clusters);
  void visualizeLidarPose(const CloudInfo& cloud_info) const;
  void visualizeLidarPoints(const Cloud& cloud) const;
  void visualizePointDetections(const Cloud& cloud,
                                const CloudInfo& cloud_info) const;
  void visualizeClusterDetections(const Cloud& cloud,
                                  const CloudInfo& cloud_info,
                                  const Clusters& clusters) const;
  void visualizeObjectDetections(const Cloud& cloud,
                                 const CloudInfo& cloud_info,
                                 const Clusters& clusters) const;
  void visualizeGroundTruth(const Cloud& cloud, const CloudInfo& cloud_info,
                            const std::string& ns = "") const;
  void visualizeMesh() const;
  void visualizeEverFree() const;
  void visualizeEverFreeSlice(const float slice_height) const;
  void visualizeTsdfSlice(const float slice_height) const;
  void visualizeSlicePoints(const Cloud& cloud,
                            const CloudInfo& cloud_info) const;
  void visualizeClusters(const Clusters& clusters,
                         const std::string& ns = "") const;

  // ROS msg helper tools.
  static geometry_msgs::msg::Vector3 setScale(const float scale);
  static std_msgs::msg::ColorRGBA setColor(const std::vector<float>& color);
  static std_msgs::msg::ColorRGBA setColor(const voxblox::Color& color);
  static geometry_msgs::msg::Point setPoint(const Point& point);
  static geometry_msgs::msg::Point setPoint(const voxblox::Point& point);

 private:
  const Config config_;
  voxblox::ExponentialOffsetIdColorMap color_map_;
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<voxblox::MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;

  // Publishers.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_points_comp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_cluster_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_cluster_comp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_object_comp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gt_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gt_cluster_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gt_object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ever_free_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr never_free_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ever_free_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr never_free_slice_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_slice_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_slice_pub_;
  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_vis_pub_;

  // Variables.
  rclcpp::Time current_stamp_;
  bool time_stamp_set_ = false;

  using MarkerTuple = std::tuple<visualization_msgs::msg::Marker, visualization_msgs::msg::Marker>;

  // Helper functions.
  MarkerTuple visualizeGroundTruthAtLevel(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const std::function<bool(const PointInfo&)>& check_level,
    const std::string& ns) const;

  rclcpp::Time getStamp() const;
};

}  // namespace dynablox

#endif  // DYNABLOX_ROS_VISUALIZATION_MOTION_VISUALIZER_H_
