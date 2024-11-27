#include "dynablox_ros/visualization/motion_visualizer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace dynablox {

MotionVisualizer::MotionVisualizer(rclcpp::Node::SharedPtr nh,
                                   std::shared_ptr<TsdfLayer> tsdf_layer,
                                   const Config& config)
      : config_(config),
        nh_(std::move(nh)),
        tsdf_layer_(std::move(tsdf_layer)) {
  color_map_.setItemsPerRevolution(config_.color_wheel_num_colors);
  // Setup mesh integrator.
  mesh_layer_ = std::make_shared<voxblox::MeshLayer>(tsdf_layer_->block_size());
  voxblox::MeshIntegratorConfig mesh_config;
  mesh_integrator_ = std::make_shared<voxblox::MeshIntegrator<TsdfVoxel>>(
      mesh_config, tsdf_layer_.get(), mesh_layer_.get());

  // Advertise topics.
  setupRos();
}

void MotionVisualizer::setupRos() {
  // Declare a QoS profile with a queue size.
  const rclcpp::QoS qos_profile(10);

  // Advertise all topics with shared pointers to publishers in ROS 2.
  sensor_pose_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("lidar_pose", qos_profile);
  sensor_points_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("lidar_points", qos_profile);
  detection_points_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("detections/point/dynamic", qos_profile);
  detection_points_comp_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("detections/point/static", qos_profile);
  detection_cluster_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("detections/cluster/dynamic", qos_profile);
  detection_cluster_comp_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("detections/cluster/static", qos_profile);
  detection_object_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("detections/object/dynamic", qos_profile);
  detection_object_comp_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("detections/object/static", qos_profile);
  gt_point_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("ground_truth/point", qos_profile);
  gt_cluster_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("ground_truth/cluster", qos_profile);
  gt_object_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("ground_truth/object", qos_profile);
  ever_free_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("ever_free", qos_profile);
  never_free_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("never_free", qos_profile);
  mesh_pub_ = nh_->create_publisher<voxblox_msgs::msg::Mesh>("mesh", qos_profile);
  ever_free_slice_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("slice/ever_free", qos_profile);
  never_free_slice_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("slice/never_free", qos_profile);
  tsdf_slice_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("slice/tsdf", qos_profile);
  point_slice_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("slice/points", qos_profile);
  cluster_vis_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", qos_profile);
}

void MotionVisualizer::visualizeAll(const Cloud& cloud,
                                    const CloudInfo& cloud_info,
                                    const Clusters& clusters) {
  current_stamp_ = rclcpp::Time(cloud_info.timestamp, RCL_SYSTEM_TIME);
  time_stamp_set_ = true;
  visualizeLidarPose(cloud_info);
  visualizeLidarPoints(cloud);
  visualizePointDetections(cloud, cloud_info);
  visualizeClusterDetections(cloud, cloud_info, clusters);
  visualizeObjectDetections(cloud, cloud_info, clusters);
  visualizeGroundTruth(cloud, cloud_info);
  visualizeMesh();
  visualizeEverFree();
  const float slice_height =
      config_.slice_relative_to_sensor
          ? config_.slice_height + cloud_info.sensor_position.z
          : config_.slice_height;
  visualizeEverFreeSlice(slice_height);
  visualizeTsdfSlice(slice_height);
  visualizeSlicePoints(cloud, cloud_info);
  visualizeClusters(clusters);
  time_stamp_set_ = false;
}

void MotionVisualizer::visualizeClusters(const Clusters& clusters,
                                         const std::string& ns) const {
  if (cluster_vis_pub_->get_subscription_count() == 0u) {
    return;
  }

  // Visualize Bbox.
  visualization_msgs::msg::MarkerArray array_msg;

  size_t id = 0;
  for (const Cluster& cluster : clusters) {
    if (cluster.points.size() > 1u) {
      visualization_msgs::msg::Marker msg;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.id = id++;
      msg.ns = ns;
      msg.header.stamp = getStamp();
      msg.header.frame_id = config_.global_frame_name;
      msg.type = visualization_msgs::msg::Marker::LINE_LIST;
      msg.color = setColor(color_map_.colorLookup(cluster.id));
      msg.scale.x = config_.cluster_line_width;
      msg.pose.orientation.w = 1.f;
      const Eigen::Vector3f base = cluster.aabb.min_corner.getVector3fMap();
      const Eigen::Vector3f delta = cluster.aabb.max_corner.getVector3fMap() - base;
      const Eigen::Vector3f dx = delta.cwiseProduct(Eigen::Vector3f::UnitX());
      const Eigen::Vector3f dy = delta.cwiseProduct(Eigen::Vector3f::UnitY());
      const Eigen::Vector3f dz = delta.cwiseProduct(Eigen::Vector3f::UnitZ());

      // All points of the box.
      msg.points.push_back(setPoint(base));
      msg.points.push_back(setPoint(base + dx));
      msg.points.push_back(setPoint(base));
      msg.points.push_back(setPoint(base + dy));
      msg.points.push_back(setPoint(base + dx));
      msg.points.push_back(setPoint(base + dx + dy));
      msg.points.push_back(setPoint(base + dy));
      msg.points.push_back(setPoint(base + dx + dy));

      msg.points.push_back(setPoint(base + dz));
      msg.points.push_back(setPoint(base + dx + dz));
      msg.points.push_back(setPoint(base + dz));
      msg.points.push_back(setPoint(base + dy + dz));
      msg.points.push_back(setPoint(base + dx + dz));
      msg.points.push_back(setPoint(base + dx + dy + dz));
      msg.points.push_back(setPoint(base + dy + dz));
      msg.points.push_back(setPoint(base + dx + dy + dz));

      msg.points.push_back(setPoint(base));
      msg.points.push_back(setPoint(base + dz));
      msg.points.push_back(setPoint(base + dx));
      msg.points.push_back(setPoint(base + dx + dz));
      msg.points.push_back(setPoint(base + dy));
      msg.points.push_back(setPoint(base + dy + dz));
      msg.points.push_back(setPoint(base + dx + dy));
      msg.points.push_back(setPoint(base + dx + dy + dz));
      array_msg.markers.push_back(msg);
    }
    visualization_msgs::msg::Marker msg;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.id = id++;
    msg.ns = ns;
    msg.header.stamp = getStamp();
    msg.header.frame_id = config_.global_frame_name;
    msg.color = setColor(color_map_.colorLookup(cluster.id));
    msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    msg.scale.z = 0.5;
    msg.pose.position = setPoint(cluster.aabb.max_corner);
    msg.pose.orientation.w = 1.f;
    const float extent = cluster.aabb.extent();
    std::stringstream stream;
    stream << cluster.points.size() << "pts - " << std::fixed
           << std::setprecision(1) << extent << "m";
    msg.text = stream.str();
    array_msg.markers.push_back(msg);
  }
  if (!array_msg.markers.empty()) {
    cluster_vis_pub_->publish(array_msg);
  }
}

void MotionVisualizer::visualizeEverFree() const {
  const bool ever_free = ever_free_pub_->get_subscription_count() > 0u;
  const bool never_free = never_free_pub_->get_subscription_count() > 0u;

  if (!ever_free && !never_free) {
    return;
  }

  visualization_msgs::msg::Marker result;
  visualization_msgs::msg::Marker result_never;

  if (ever_free) {
    // Common properties.
    result.action = visualization_msgs::msg::Marker::ADD;
    result.id = 0;
    result.header.stamp = getStamp();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::msg::Marker::CUBE_LIST;
    result.color = setColor(config_.ever_free_color);
    result.scale = setScale(tsdf_layer_->voxel_size());
    result.pose.orientation.w = 1.f;
  }

  if (never_free) {
    result_never.action = visualization_msgs::msg::Marker::ADD;
    result_never.id = 0;
    result_never.header.stamp = getStamp();
    result_never.header.frame_id = config_.global_frame_name;
    result_never.type = visualization_msgs::msg::Marker::CUBE_LIST;
    result_never.color = setColor(config_.never_free_color);
    result_never.scale = setScale(tsdf_layer_->voxel_size());
    result_never.pose.orientation.w = 1.f;
  }

  voxblox::BlockIndexList block_list;
  tsdf_layer_->getAllAllocatedBlocks(&block_list);
  for (const auto& index : block_list) {
    const TsdfBlock& block = tsdf_layer_->getBlockByIndex(index);
    for (size_t linear_index = 0; linear_index < block.num_voxels();
         ++linear_index) {
      const TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      if (voxel.weight < 1e-6) {
        continue;  // Unknown voxel.
      }

      const voxblox::Point coords =
          block.computeCoordinatesFromLinearIndex(linear_index);
      if (coords.z() > config_.visualization_max_z) {
        continue;
      }

      if (voxel.ever_free && ever_free) {
        result.points.push_back(setPoint(coords));
      } else if (!voxel.ever_free && never_free) {
        result_never.points.push_back(setPoint(coords));
      }
    }
  }

  if (!result.points.empty()) {
    ever_free_pub_->publish(result);
  }

  if (!result_never.points.empty()) {
    never_free_pub_->publish(result_never);
  }
}

void MotionVisualizer::visualizeEverFreeSlice(const float slice_height) const {
  const bool ever_free = ever_free_slice_pub_->get_subscription_count() > 0u;
  const bool never_free = never_free_slice_pub_->get_subscription_count() > 0u;

  if (!ever_free && !never_free) {
    return;
  }

  visualization_msgs::msg::Marker result;
  visualization_msgs::msg::Marker result_never;

  if (ever_free) {
    // Common properties.
    result.action = visualization_msgs::msg::Marker::ADD;
    result.id = 0;
    result.header.stamp = getStamp();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::msg::Marker::CUBE_LIST;
    result.color = setColor(config_.ever_free_color);
    result.scale = setScale(tsdf_layer_->voxel_size());
    result.scale.z = 0.01f;
    result.pose.orientation.w = 1.f;
  }

  if (never_free) {
    result_never.action = visualization_msgs::msg::Marker::ADD;
    result_never.id = 0;
    result_never.header.stamp = getStamp();
    result_never.header.frame_id = config_.global_frame_name;
    result_never.type = visualization_msgs::msg::Marker::CUBE_LIST;
    result_never.color = setColor(config_.never_free_color);
    result_never.scale = setScale(tsdf_layer_->voxel_size());
    result_never.scale.z = 0.01f;
    result_never.pose.orientation.w = 1.f;
  }

  // Setup the slice.
  const voxblox::Point slice_coords(0, 0, slice_height);
  const BlockIndex slice_block_index =
      tsdf_layer_->computeBlockIndexFromCoordinates(slice_coords);
  const VoxelIndex slice_voxel_index =
      voxblox::getGridIndexFromPoint<VoxelIndex>(
          slice_coords - voxblox::getOriginPointFromGridIndex(
                             slice_block_index, tsdf_layer_->block_size()),
          tsdf_layer_->voxel_size_inv());

  // Visualize.
  voxblox::BlockIndexList block_list;
  tsdf_layer_->getAllAllocatedBlocks(&block_list);
  const float offset = tsdf_layer_->voxel_size() / 2.f;
  for (const auto& index : block_list) {
    if (index.z() != slice_block_index.z()) {
      continue;
    }
    const TsdfBlock& block = tsdf_layer_->getBlockByIndex(index);
    for (size_t x = 0; x < block.voxels_per_side(); ++x) {
      for (size_t y = 0; y < block.voxels_per_side(); ++y) {
        const VoxelIndex index(x, y, slice_voxel_index.z());
        const TsdfVoxel& voxel = block.getVoxelByVoxelIndex(index);

        if (voxel.weight < 1e-6) {
          continue;  // Unknown voxel.
        }

        voxblox::Point coords = block.computeCoordinatesFromVoxelIndex(index);
        coords.z() -= offset;

        if (voxel.ever_free && ever_free) {
          result.points.push_back(setPoint(coords));
        } else if ((!voxel.ever_free) && never_free) {
          result_never.points.push_back(setPoint(coords));
        }
      }
    }
  }

  if (!result.points.empty()) {
    ever_free_slice_pub_->publish(result);
  }

  if (!result_never.points.empty()) {
    never_free_slice_pub_->publish(result_never);
  }
}

void MotionVisualizer::visualizeTsdfSlice(const float slice_height) const {
  if (tsdf_slice_pub_->get_subscription_count() == 0u) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  voxblox::createDistancePointcloudFromTsdfLayerSlice(
      *tsdf_layer_, 2u, slice_height, &pointcloud);

  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(pointcloud, ros_pointcloud);

  ros_pointcloud.header.frame_id = config_.global_frame_name;
  ros_pointcloud.header.stamp = getStamp(); // Could be very wrong?
  tsdf_slice_pub_->publish(ros_pointcloud);
}

void MotionVisualizer::visualizeSlicePoints(const Cloud& cloud,
                                            const CloudInfo& cloud_info) const {
  if (point_slice_pub_->get_subscription_count() == 0u) {
    return;
  }

  visualization_msgs::msg::Marker result;
  result.action = visualization_msgs::msg::Marker::ADD;
  result.id = 0;
  result.header.stamp = getStamp();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::msg::Marker::POINTS;
  result.scale = setScale(config_.dynamic_point_scale);

  visualization_msgs::msg::Marker result_comp;
  result_comp.action = visualization_msgs::msg::Marker::ADD;
  result_comp.id = 1;
  result_comp.header.stamp = getStamp();
  result_comp.header.frame_id = config_.global_frame_name;
  result_comp.type = visualization_msgs::msg::Marker::POINTS;
  result_comp.color = setColor(config_.static_point_color);
  result_comp.scale = setScale(config_.static_point_scale);

  const float slice_height =
      config_.slice_relative_to_sensor
          ? config_.slice_height + cloud_info.sensor_position.z
          : config_.slice_height;
  const float slice_center =
      std::round((slice_height * tsdf_layer_->voxel_size_inv()) + 0.5) *
      tsdf_layer_->voxel_size();
  const float min_z = slice_center - tsdf_layer_->voxel_size() / 2.f;
  const float max_z = slice_center + tsdf_layer_->voxel_size() / 2.f;

  // Get all points.
  int i = -1;
  for (const auto& point : cloud.points) {
    ++i;
    if (point.z < min_z || point.z > max_z) {
      continue;
    }
    if (cloud_info.points[i].ever_free_level_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.point_level_slice_color));
    } else if (cloud_info.points[i].cluster_level_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.cluster_level_slice_color));
    } else {
      result_comp.points.push_back(setPoint(point));
    }
  }
  if (!result.points.empty()) {
    point_slice_pub_->publish(result);
  }
  if (!result_comp.points.empty()) {
    point_slice_pub_->publish(result_comp);
  }
}

void MotionVisualizer::visualizeGroundTruth(const Cloud& cloud,
                                            const CloudInfo& cloud_info,
                                            const std::string& ns) const {
  if (!cloud_info.has_labels) {
    return;
  }
  // Go through all levels if it has subscribers.
  if (gt_point_pub_->get_subscription_count() > 0) {
    auto [result, comp] = visualizeGroundTruthAtLevel(cloud, cloud_info,
        [](const PointInfo& point) { return point.ever_free_level_dynamic; }, ns);
    gt_point_pub_->publish(result);
    gt_point_pub_->publish(comp);
  }
  if (gt_cluster_pub_->get_subscription_count() > 0) {
    auto [result, comp] = visualizeGroundTruthAtLevel(cloud, cloud_info,
        [](const PointInfo& point) { return point.cluster_level_dynamic; }, ns);
    gt_cluster_pub_->publish(result);
    gt_cluster_pub_->publish(comp);
  }
  if (gt_object_pub_->get_subscription_count() > 0) {
    auto [result, comp] = visualizeGroundTruthAtLevel(cloud, cloud_info,
        [](const PointInfo& point) { return point.object_level_dynamic; }, ns);
    gt_object_pub_->publish(result);
    gt_object_pub_->publish(comp);
  }
}

MotionVisualizer::MarkerTuple MotionVisualizer::visualizeGroundTruthAtLevel(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const std::function<bool(const PointInfo&)>& check_level,
    const std::string& ns) const {
  // Common properties.
  visualization_msgs::msg::Marker result;
  result.action = visualization_msgs::msg::Marker::ADD;
  result.id = 0;
  result.ns = ns;
  result.header.stamp = getStamp();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::msg::Marker::POINTS;
  result.scale = setScale(config_.dynamic_point_scale);

  visualization_msgs::msg::Marker comp = result;
  comp.scale = setScale(config_.static_point_scale);
  comp.id = 1;

  // Get all points.
  size_t i = 0;
  for (const auto& point : cloud.points) {
    const PointInfo& info = cloud_info.points[i];
    ++i;
    if (point.z > config_.visualization_max_z) {
      continue;
    }
    if (!info.ready_for_evaluation) {
      comp.points.push_back(setPoint(point));
      comp.colors.push_back(setColor(config_.out_of_bounds_color));
    } else if (check_level(info) && info.ground_truth_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.true_positive_color));
    } else if (check_level(info) && !info.ground_truth_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.false_positive_color));
    } else if (!check_level(info) && info.ground_truth_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.false_negative_color));
    } else {
      comp.points.push_back(setPoint(point));
      comp.colors.push_back(setColor(config_.true_negative_color));
    }
  }
  return std::make_tuple(result, comp);
}

void MotionVisualizer::visualizeLidarPose(const CloudInfo& cloud_info) const {
  if (sensor_pose_pub_->get_subscription_count() == 0u) {
    return;
  }
  visualization_msgs::msg::Marker result;
  result.action = visualization_msgs::msg::Marker::ADD;
  result.id = 0;
  result.header.stamp = getStamp();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::msg::Marker::SPHERE;
  result.color = setColor(config_.sensor_color);
  result.scale = setScale(config_.sensor_scale);
  result.pose.position = setPoint(cloud_info.sensor_position);
  result.pose.orientation.w = 1.0;
  sensor_pose_pub_->publish(result);
}

void MotionVisualizer::visualizeLidarPoints(const Cloud& cloud) const {
  if (sensor_points_pub_->get_subscription_count() == 0u) {
    return;
  }
  visualization_msgs::msg::Marker result;
  result.points.reserve(cloud.points.size());

  // Common properties.
  result.action = visualization_msgs::msg::Marker::ADD;
  result.id = 0;
  result.header.stamp = getStamp();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::msg::Marker::POINTS;
  result.color = setColor(config_.static_point_color);
  result.scale = setScale(config_.static_point_scale);

  // Get all points.
  for (const auto& point : cloud.points) {
    if (point.z > config_.visualization_max_z) {
      continue;
    }
    result.points.push_back(setPoint(point));
  }
  if (!result.points.empty()) {
    sensor_points_pub_->publish(result);
  }
}

void MotionVisualizer::visualizePointDetections(
    const Cloud& cloud, const CloudInfo& cloud_info) const {
  const bool dynamic = detection_points_pub_->get_subscription_count() > 0u;
  const bool comp = detection_points_comp_pub_->get_subscription_count() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::msg::Marker result;
  visualization_msgs::msg::Marker result_comp;

  if (dynamic) {
    // Common properties.
    result.points.reserve(cloud.points.size());
    result.action = visualization_msgs::msg::Marker::ADD;
    result.id = 0;
    result.header.stamp = getStamp();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::msg::Marker::POINTS;
    result.color = setColor(config_.dynamic_point_color);
    result.scale = setScale(config_.dynamic_point_scale);
  }

  if (comp) {
    result_comp.points.reserve(cloud.points.size());
    result_comp.action = visualization_msgs::msg::Marker::ADD;
    result_comp.id = 0;
    result_comp.header.stamp = getStamp();
    result_comp.header.frame_id = config_.global_frame_name;
    result_comp.type = visualization_msgs::msg::Marker::POINTS;
    result_comp.color = setColor(config_.static_point_color);
    result_comp.scale = setScale(config_.static_point_scale);
  }

  // Get all points.
  int i = -1;
  for (const auto& point : cloud.points) {
    ++i;
    if (point.z > config_.visualization_max_z) {
      continue;
    }
    if (cloud_info.points[i].ever_free_level_dynamic) {
      if (!dynamic) {
        continue;
      }
      result.points.push_back(setPoint(point));
    } else {
      if (!comp) {
        continue;
      }
      result_comp.points.push_back(setPoint(point));
    }
  }
  if (!result.points.empty()) {
    detection_points_pub_->publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_points_comp_pub_->publish(result_comp);
  }
}

void MotionVisualizer::visualizeClusterDetections(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const Clusters& clusters) const {
  const bool dynamic = detection_cluster_pub_->get_subscription_count() > 0u;
  const bool comp = detection_cluster_comp_pub_->get_subscription_count() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::msg::Marker result;
  visualization_msgs::msg::Marker result_comp;

  if (dynamic) {
    // We just reserve too much space to save compute.
    result.points.reserve(cloud.points.size());
    result.action = visualization_msgs::msg::Marker::ADD;
    result.id = 0;
    result.header.stamp = getStamp();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::msg::Marker::POINTS;
    result.scale = setScale(config_.dynamic_point_scale);
  }

  if (comp) {
    result_comp.points.reserve(cloud.points.size());
    result_comp.action = visualization_msgs::msg::Marker::ADD;
    result_comp.id = 0;
    result_comp.header.stamp = getStamp();
    result_comp.header.frame_id = config_.global_frame_name;
    result_comp.type = visualization_msgs::msg::Marker::POINTS;
    result_comp.color = setColor(config_.static_point_color);
    result_comp.scale = setScale(config_.static_point_scale);
  }

  // Get all cluster points.
  int i = 0;
  for (const Cluster& cluster : clusters) {
    std_msgs::msg::ColorRGBA color;
    if (config_.color_clusters) {
      color = setColor(color_map_.colorLookup(i));
      ++i;
    } else {
      color = setColor(config_.dynamic_point_color);
    }
    for (int index : cluster.points) {
      if (          cloud[index].z > config_.visualization_max_z) {
        continue;
      }
      result.points.push_back(setPoint(cloud[index]));
      result.colors.push_back(color);
    }
  }

  // Get all other points.
  if (comp) {
    size_t i = 0;
    for (const auto& point : cloud.points) {
      if (point.z > config_.visualization_max_z) {
        ++i;
        continue;
      }
      if (!cloud_info.points[i].cluster_level_dynamic) {
        result_comp.points.push_back(setPoint(point));
      }
      ++i;
    }
  }

  if (!result.points.empty()) {
    detection_cluster_pub_->publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_cluster_comp_pub_->publish(result_comp);
  }
}

void MotionVisualizer::visualizeObjectDetections(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const Clusters& clusters) const {
  // TODO(schmluk): This is currently copied from the clusters, it simply tries
  // to do color associations for a bit more consistency during visualization.
  const bool dynamic = detection_object_pub_->get_subscription_count() > 0u;
  const bool comp = detection_object_comp_pub_->get_subscription_count() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::msg::Marker result;
  visualization_msgs::msg::Marker result_comp;

  if (dynamic) {
    // We just reserve too much space to save compute.
    result.points.reserve(cloud.points.size());
    result.action = visualization_msgs::msg::Marker::ADD;
    result.id = 0;
    result.header.stamp = getStamp();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::msg::Marker::POINTS;
    result.scale = setScale(config_.dynamic_point_scale);
  }

  if (comp) {
    result_comp = result;
    result_comp.color = setColor(config_.static_point_color);
    result_comp.scale = setScale(config_.static_point_scale);
  }

  // Get all cluster points.
  for (const Cluster& cluster : clusters) {
    if (!cluster.valid) {
      continue;
    }
    std_msgs::msg::ColorRGBA color;
    if (config_.color_clusters) {
      color = setColor(color_map_.colorLookup(cluster.id));
    } else {
      color = setColor(config_.dynamic_point_color);
    }

    for (int index : cluster.points) {
      if (          cloud[index].z > config_.visualization_max_z) {
        continue;
      }
      result.points.push_back(setPoint(cloud[index]));
      result.colors.push_back(color);
    }
  }

  // Get all other points.
  if (comp) {
    size_t i = 0;
    for (const auto& point : cloud.points) {
      if (point.z > config_.visualization_max_z) {
        ++i;
        continue;
      }
      if (!cloud_info.points[i].object_level_dynamic) {
        result_comp.points.push_back(setPoint(point));
      }
      ++i;
    }
  }

  if (!result.points.empty()) {
    detection_object_pub_->publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_object_comp_pub_->publish(result_comp);
  }
}

void MotionVisualizer::visualizeMesh() const {
  if (mesh_pub_->get_subscription_count() == 0u) {
    return;
  }
  mesh_integrator_->generateMesh(true, true);
  voxblox_msgs::msg::Mesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh_layer_, voxblox::ColorMode::kLambert,
                                  &mesh_msg);
  mesh_msg.header.frame_id = config_.global_frame_name;
  mesh_msg.header.stamp = getStamp();
  mesh_pub_->publish(mesh_msg);
}

geometry_msgs::msg::Vector3 MotionVisualizer::setScale(const float scale) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = scale;
  msg.y = scale;
  msg.z = scale;
  return msg;
}

std_msgs::msg::ColorRGBA MotionVisualizer::setColor(const std::vector<double>& color) {
  std_msgs::msg::ColorRGBA msg;
  msg.r = color[0];
  msg.g = color[1];
  msg.b = color[2];
  msg.a = color[3];
  return msg;
}

std_msgs::msg::ColorRGBA MotionVisualizer::setColor(const voxblox::Color& color) {
  std_msgs::msg::ColorRGBA msg;
  msg.r = static_cast<float>(color.r) / 255.f;
  msg.g = static_cast<float>(color.g) / 255.f;
  msg.b = static_cast<float>(color.b) / 255.f;
  msg.a = static_cast<float>(color.a) / 255.f;
  return msg;
}

geometry_msgs::msg::Point MotionVisualizer::setPoint(const Point& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

geometry_msgs::msg::Point MotionVisualizer::setPoint(const voxblox::Point& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

rclcpp::Time MotionVisualizer::getStamp() const {
  if (time_stamp_set_) {
    return current_stamp_;
  } else {
    return nh_->get_clock()->now();
  }
}

}  // namespace dynablox
