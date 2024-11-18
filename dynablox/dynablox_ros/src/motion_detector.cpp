#include "dynablox_ros/motion_detector.h"

#include <math.h>

#include <future>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <minkindr/kindr_tf.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>

namespace dynablox {

using Timer = voxblox::timing::Timer;

MotionDetector::MotionDetector(const rclcpp::Node::SharedPtr& nh,
                               const rclcpp::Node::SharedPtr& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      tf_buffer_(nh_private_->get_clock()),
      tf_listener_(tf_buffer_) {
  setupMembers();

  // Cache frequently used constants.
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;

  // Advertise and subscribe to topics.
  setupRos();
}

void MotionDetector::setupMembers() {
// Create a new node for voxblox parameters
  auto nh_voxblox = std::make_shared<rclcpp::Node>("voxblox", nh_private_->get_namespace());

  // Declare and set parameters for the TSDF server (Voxblox settings).
  nh_voxblox->declare_parameter("world_frame", config_.global_frame_name);
  nh_voxblox->declare_parameter("update_mesh_every_n_sec", 0);
  nh_voxblox->declare_parameter("voxel_carving_enabled", true);
  nh_voxblox->declare_parameter("allow_clear", true);
  nh_voxblox->declare_parameter("integrator_threads", config_.num_threads);

  // Initialize the TSDF server with parameters from the new voxblox node.
  tsdf_server_ = std::make_shared<voxblox::TsdfServer>(nh_voxblox, nh_voxblox);
  tsdf_layer_.reset(tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr());

  // Preprocessing.
  preprocessing_ = std::make_shared<Preprocessing>(
      config_utilities::getConfigFromNode<Preprocessing::Config>(
          nh_private_, "preprocessing"));

  // Clustering.
  clustering_ = std::make_shared<Clustering>(
      config_utilities::getConfigFromNode<Clustering::Config>(
          nh_private_, "clustering"),
      tsdf_layer_);

  // Tracking.
  tracking_ = std::make_shared<Tracking>(
      config_utilities::getConfigFromNode<Tracking::Config>(
          nh_private_, "tracking"));

  // Ever-Free Integrator.
  nh_private_->declare_parameter("ever_free_integrator.num_threads", config_.num_threads);
  ever_free_integrator_ = std::make_shared<EverFreeIntegrator>(
      config_utilities::getConfigFromNode<EverFreeIntegrator::Config>(
          nh_private_, "ever_free_integrator"), tsdf_layer_);

  // Evaluation.
  if (config_.evaluate) {
    // Initialize evaluator if evaluation is requested.
    evaluator_ = std::make_shared<Evaluator>(
        config_utilities::getConfigFromNode<Evaluator::Config>(
            nh_private_, "evaluation"));
  }

  // Visualization.
  visualizer_ = std::make_shared<MotionVisualizer>(nh_private_, tsdf_layer_, "visualization");
}

void MotionDetector::setupRos() {
  lidar_pcl_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::QoS(config_.queue_size),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->pointcloudCallback(msg);
    });
}

void MotionDetector::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  Timer frame_timer("frame");
  Timer detection_timer("motion_detection");

  // Lookup cloud transform T_M_S of sensor (S) to map (M).
  // If different sensor frame is required, update the message.
  Timer tf_lookup_timer("motion_detection/tf_lookup");
  const std::string sensor_frame_name = config_.sensor_frame_name.empty()
                                            ? msg->header.frame_id
                                            : config_.sensor_frame_name;

  geometry_msgs::msg::TransformStamped T_M_S;
  if (!lookupTransform(config_.global_frame_name, sensor_frame_name,
                       rclcpp::Time(msg->header.stamp).nanoseconds(), T_M_S)) {
    // Getting transform failed, need to skip.
    return;
  }
  tf_lookup_timer.Stop();

  // Preprocessing.
  Timer preprocessing_timer("motion_detection/preprocessing");
  frame_counter_++;
  CloudInfo cloud_info;
  Cloud cloud;

  preprocessing_->processPointcloud(msg, T_M_S, cloud, cloud_info, rclcpp::Time(msg->header.stamp).nanoseconds()); // What do I do here?
  preprocessing_timer.Stop();

  // Build a mapping of all blocks to voxels to points for the scan.
  Timer setup_timer("motion_detection/indexing_setup");
  BlockToPointMap point_map;
  std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices;
  setUpPointMap(cloud, point_map, occupied_ever_free_voxel_indices, cloud_info);
  setup_timer.Stop();

  // Clustering.
  Timer clustering_timer("motion_detection/clustering");
  Clusters clusters = clustering_->performClustering(
      point_map, occupied_ever_free_voxel_indices, frame_counter_, cloud,
      cloud_info);
  clustering_timer.Stop();

  // Tracking.
  Timer tracking_timer("motion_detection/tracking");
  tracking_->track(cloud, clusters, cloud_info);
  tracking_timer.Stop();

  // Integrate ever-free information.
  Timer update_ever_free_timer("motion_detection/update_ever_free");
  ever_free_integrator_->updateEverFreeVoxels(frame_counter_);
  update_ever_free_timer.Stop();

  // Integrate the pointcloud into the voxblox TSDF map.
  Timer tsdf_timer("motion_detection/tsdf_integration");
  voxblox::Transformation T_G_C;
  tf2::transformTFToKindr(T_M_S, &T_G_C);
  tsdf_server_->processPointCloudMessageAndInsert(msg, T_G_C, false);
  tsdf_timer.Stop();
  detection_timer.Stop();

  // Evaluation if requested.
  if (config_.evaluate) {
    Timer eval_timer("evaluation");
    evaluator_->evaluateFrame(cloud, cloud_info, clusters);
    eval_timer.Stop();
    if (config_.shutdown_after > 0 &&
        evaluator_->getNumberOfEvaluatedFrames() >= config_.shutdown_after) {
      LOG(INFO) << "Evaluated " << config_.shutdown_after
                << " frames, shutting down";
      rclcpp::shutdown();
    }
  }

  // Visualization if requested.
  if (config_.visualize) {
    Timer vis_timer("visualizations");
    visualizer_->visualizeAll(cloud, cloud_info, clusters);
    vis_timer.Stop();
  }
}

bool MotionDetector::lookupTransform(const std::string& target_frame,
                                     const std::string& source_frame,
                                     uint64_t timestamp,
                                     geometry_msgs::msg::TransformStamped& result) const {
  // Convert the timestamp to rclcpp::Time
  rclcpp::Time timestamp_ros(timestamp);

  try {
    // Use the lookupTransform method from the tf_buffer
    result = tf_buffer_.lookupTransform(target_frame, source_frame, timestamp_ros);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(nh_private_->get_logger(), "Could not get sensor transform, skipping pointcloud: %s", ex.what());
    return false;
  }
  return true;
}

void MotionDetector::setUpPointMap(
    const Cloud& cloud, BlockToPointMap& point_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    CloudInfo& cloud_info) const {
  // Identifies for any LiDAR point the block it falls in and constructs the
  // hash-map block2points_map mapping each block to the LiDAR points that
  // fall into the block.
  const voxblox::HierarchicalIndexIntMap block2points_map =
      buildBlockToPointsMap(cloud);

  // Builds the voxel2point-map in parallel blockwise.
  std::vector<BlockIndex> block_indices(block2points_map.size());
  size_t i = 0;
  for (const auto& block : block2points_map) {
    block_indices[i] = block.first;
    ++i;
  }
  IndexGetter<BlockIndex> index_getter(block_indices);
  std::vector<std::future<void>> threads;
  std::mutex aggregate_results_mutex;
  for (int i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      // Data to store results.
      BlockIndex block_index;
      std::vector<voxblox::VoxelKey> local_occupied_indices;
      BlockToPointMap local_point_map;

      // Process until no more blocks.
      while (index_getter.getNextIndex(&block_index)) {
        VoxelToPointMap result;
        this->blockwiseBuildPointMap(cloud, block_index,
                                     block2points_map.at(block_index), result,
                                     local_occupied_indices, cloud_info);
        local_point_map.insert(std::pair(block_index, result));
      }

      // After processing is done add data to the output map.
      std::lock_guard<std::mutex> lock(aggregate_results_mutex);
      occupied_ever_free_voxel_indices.insert(
          occupied_ever_free_voxel_indices.end(),
          local_occupied_indices.begin(), local_occupied_indices.end());
      point_map.merge(local_point_map);
    }));
  }

  for (auto& thread : threads) {
    thread.get();
  }
}

voxblox::HierarchicalIndexIntMap MotionDetector::buildBlockToPointsMap(
    const Cloud& cloud) const {
  voxblox::HierarchicalIndexIntMap result;

  int i = 0;
  for (const Point& point : cloud) {
    voxblox::Point coord(point.x, point.y, point.z);
    const BlockIndex blockindex =
        tsdf_layer_->computeBlockIndexFromCoordinates(coord);
    result[blockindex].push_back(i);
    i++;
  }
  return result;
}

void MotionDetector::blockwiseBuildPointMap(
    const Cloud& cloud, const BlockIndex& block_index,
    const voxblox::AlignedVector<size_t>& points_in_block,
    VoxelToPointMap& voxel_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    CloudInfo& cloud_info) const {
  // Get the block.
  TsdfBlock::Ptr tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
  if (!tsdf_block) {
    return;
  }

  // Create a mapping of each voxel index to the points it contains.
  for (size_t i : points_in_block) {
    const Point& point = cloud[i];
    const voxblox::Point coords(point.x, point.y, point.z);
    const VoxelIndex voxel_index =
        tsdf_block->computeVoxelIndexFromCoordinates(coords);
    if (!tsdf_block->isValidVoxelIndex(voxel_index)) {
      continue;
    }
    voxel_map[voxel_index].push_back(i);

    // EverFree detection flag at the same time, since we anyways lookup
    // voxels.
    if (tsdf_block->getVoxelByVoxelIndex(voxel_index).ever_free) {
      cloud_info.points.at(i).ever_free_level_dynamic = true;
    }
  }

  // Update the voxel status of the currently occupied voxels.
  for (const auto& voxel_points_pair : voxel_map) {
    TsdfVoxel& tsdf_voxel =
        tsdf_block->getVoxelByVoxelIndex(voxel_points_pair.first);
    tsdf_voxel.last_lidar_occupied = frame_counter_;

    // This voxel attribute is used in the voxel clustering method: it
    // signalizes that a currently occupied voxel has not yet been clustered
    tsdf_voxel.clustering_processed = false;

    // The set of occupied_ever_free_voxel_indices allows for fast access of
    // the seed voxels in the voxel clustering
    if (tsdf_voxel.ever_free) {
      occupied_ever_free_voxel_indices.push_back(
          std::make_pair(block_index, voxel_points_pair.first));
    }
  }
}

Eigen::Matrix4f MotionDetector::transformStampedToMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Extract translation
  transform(0, 3) = transform_stamped.transform.translation.x;
  transform(1, 3) = transform_stamped.transform.translation.y;
  transform(2, 3) = transform_stamped.transform.translation.z;

  // Extract rotation and convert to Eigen quaternion
  Eigen::Quaternionf q(
      transform_stamped.transform.rotation.w,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z
  );

  // Convert quaternion to rotation matrix and place it in the top-left 3x3 part
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();

  return transform;
}

}  // namespace dynablox
