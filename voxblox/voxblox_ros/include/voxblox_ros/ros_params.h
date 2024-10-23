#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <rclcpp/rclcpp.hpp>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

inline TsdfMap::Config getTsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfMap::Config tsdf_config;

  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  node->declare_parameter<double>("tsdf_voxel_size", voxel_size);
  node->declare_parameter<int>("tsdf_voxels_per_side", voxels_per_side);

  voxel_size = node->get_parameter("tsdf_voxel_size").get_value<double>();
  voxels_per_side = node->get_parameter("tsdf_voxels_per_side").get_value<int>();

  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(node->get_logger(), "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam(const rclcpp::Node::SharedPtr& node) {
  ICP::Config icp_config;

  node->declare_parameter<double>("icp_min_match_ratio", icp_config.min_match_ratio);
  node->declare_parameter<double>("icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
  node->declare_parameter<int>("icp_mini_batch_size", icp_config.mini_batch_size);
  node->declare_parameter<bool>("icp_refine_roll_pitch", icp_config.refine_roll_pitch);
  node->declare_parameter<double>("icp_inital_translation_weighting", icp_config.inital_translation_weighting);
  node->declare_parameter<double>("icp_inital_rotation_weighting", icp_config.inital_rotation_weighting);

  icp_config.min_match_ratio = node->get_parameter("icp_min_match_ratio").get_value<double>();
  icp_config.subsample_keep_ratio = node->get_parameter("icp_subsample_keep_ratio").get_value<double>();
  icp_config.mini_batch_size = node->get_parameter("icp_mini_batch_size").get_value<int>();
  icp_config.refine_roll_pitch = node->get_parameter("icp_refine_roll_pitch").get_value<bool>();
  icp_config.inital_translation_weighting = node->get_parameter("icp_inital_translation_weighting").get_value<double>();
  icp_config.inital_rotation_weighting = node->get_parameter("icp_inital_rotation_weighting").get_value<double>();

  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(const rclcpp::Node::SharedPtr& node) {
    TsdfIntegratorBase::Config integrator_config;

    integrator_config.voxel_carving_enabled = true;

    const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
    integrator_config.default_truncation_distance = tsdf_config.tsdf_voxel_size * 4;

    double truncation_distance = integrator_config.default_truncation_distance;
    double max_weight = integrator_config.max_weight;
    int integrator_threads = static_cast<int>(integrator_config.integrator_threads);

    node->declare_parameter<bool>("voxel_carving_enabled", integrator_config.voxel_carving_enabled);
    node->declare_parameter<double>("truncation_distance", truncation_distance);
    node->declare_parameter<double>("max_ray_length_m", integrator_config.max_ray_length_m);
    node->declare_parameter<double>("min_ray_length_m", integrator_config.min_ray_length_m);
    node->declare_parameter<double>("max_weight", max_weight);
    node->declare_parameter<bool>("use_const_weight", integrator_config.use_const_weight);
    node->declare_parameter<bool>("use_weight_dropoff", integrator_config.use_weight_dropoff);
    node->declare_parameter<bool>("allow_clear", integrator_config.allow_clear);
    node->declare_parameter<int>("start_voxel_subsampling_factor", integrator_config.start_voxel_subsampling_factor);
    node->declare_parameter<int>("max_consecutive_ray_collisions", integrator_config.max_consecutive_ray_collisions);
    node->declare_parameter<int>("clear_checks_every_n_frames", integrator_config.clear_checks_every_n_frames);
    node->declare_parameter<double>("max_integration_time_s", integrator_config.max_integration_time_s);
    node->declare_parameter<bool>("anti_grazing", integrator_config.enable_anti_grazing);
    node->declare_parameter<bool>("use_sparsity_compensation_factor", integrator_config.use_sparsity_compensation_factor);
    node->declare_parameter<double>("sparsity_compensation_factor", integrator_config.sparsity_compensation_factor);
    node->declare_parameter<int>("integration_order_mode", integrator_config.integration_order_mode);
    node->declare_parameter<int>("integrator_threads", integrator_threads);
    node->declare_parameter<int>("sensor_horizontal_resolution", integrator_config.sensor_horizontal_resolution);
    node->declare_parameter<int>("sensor_vertical_resolution", integrator_config.sensor_vertical_resolution);
    node->declare_parameter<double>("sensor_vertical_field_of_view_degrees", integrator_config.sensor_vertical_field_of_view_degrees);

    integrator_config.voxel_carving_enabled = node->get_parameter("voxel_carving_enabled").get_value<bool>();
    truncation_distance = node->get_parameter("truncation_distance").get_value<double>();
    integrator_config.max_ray_length_m = node->get_parameter("max_ray_length_m").get_value<double>();
    integrator_config.min_ray_length_m = node->get_parameter("min_ray_length_m").get_value<double>();
    max_weight = node->get_parameter("max_weight").get_value<double>();
    integrator_config.use_const_weight = node->get_parameter("use_const_weight").get_value<bool>();
    integrator_config.use_weight_dropoff = node->get_parameter("use_weight_dropoff").get_value<bool>();
    integrator_config.allow_clear = node->get_parameter("allow_clear").get_value<bool>();
    integrator_config.start_voxel_subsampling_factor = node->get_parameter("start_voxel_subsampling_factor").get_value<int>();
    integrator_config.max_consecutive_ray_collisions = node->get_parameter("max_consecutive_ray_collisions").get_value<int>();
    integrator_config.clear_checks_every_n_frames = node->get_parameter("clear_checks_every_n_frames").get_value<int>();
    integrator_config.max_integration_time_s = node->get_parameter("max_integration_time_s").get_value<double>();
    integrator_config.enable_anti_grazing = node->get_parameter("anti_grazing").get_value<bool>();
    integrator_config.use_sparsity_compensation_factor = node->get_parameter("use_sparsity_compensation_factor").get_value<bool>();
    integrator_config.sparsity_compensation_factor = node->get_parameter("sparsity_compensation_factor").get_value<double>();
    integrator_config.integration_order_mode = node->get_parameter("integration_order_mode").get_value<int>();
    integrator_threads = node->get_parameter("integrator_threads").get_value<int>();
    integrator_config.sensor_horizontal_resolution = node->get_parameter("sensor_horizontal_resolution").get_value<int>();
    integrator_config.sensor_vertical_resolution = node->get_parameter("sensor_vertical_resolution").get_value<int>();
    integrator_config.sensor_vertical_field_of_view_degrees = node->get_parameter("sensor_vertical_field_of_view_degrees").get_value<double>();

    integrator_config.default_truncation_distance = static_cast<float>(truncation_distance);
    integrator_config.max_weight = static_cast<float>(max_weight);
    integrator_config.integrator_threads = integrator_threads;

    return integrator_config;
}


inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  MeshIntegratorConfig mesh_integrator_config;

  node->declare_parameter<double>("mesh_min_weight", mesh_integrator_config.min_weight);
  node->declare_parameter<bool>("mesh_use_color", mesh_integrator_config.use_color);

  mesh_integrator_config.min_weight = node->get_parameter("mesh_min_weight").get_value<double>();
  mesh_integrator_config.use_color = node->get_parameter("mesh_use_color").get_value<bool>();

  return mesh_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_
