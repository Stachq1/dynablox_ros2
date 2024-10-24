#ifndef VOXBLOX_ROS_TRANSFORMER_H_
#define VOXBLOX_ROS_TRANSFORMER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_listener.h>

#include <voxblox/core/common.h>

namespace voxblox {

/**
 * Class that binds to either the TF tree or resolves transformations from the
 * ROS parameter server, depending on settings loaded from ROS params.
 */
class Transformer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transformer(const rclcpp::Node::SharedPtr& nh, const rclcpp::Node::SharedPtr& nh_private);

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const rclcpp::Time& timestamp,
                       Transformation* transform);

 private:
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const rclcpp::Time& timestamp, Transformation* transform);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr nh_private_;

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string world_frame_;
  /// If set, overwrite sensor frame with this value. If empty, unused.
  std::string sensor_frame_;
  /**
   * Whether to use TF transform resolution (true) or fixed transforms from
   * parameters and transform topics (false).
   */
  int64_t timestamp_tolerance_ns_;

  /**
   * If we use topic transforms, we have 2 parts: a dynamic transform from a
   * topic and a static transform from parameters.
   * Static transform should be T_G_D (where D is whatever sensor the
   * dynamic coordinate frame is in) and the static should be T_D_C (where
   * C is the sensor frame that produces the depth data). It is possible to
   * specify T_C_D and set invert_static_transform to true.
   */

  /**
   * To be replaced (at least optionally) with odometry + static transform
   * from IMU to visual frame.
   */
  tf2::TransformListener tf_listener_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TRANSFORMER_H_
