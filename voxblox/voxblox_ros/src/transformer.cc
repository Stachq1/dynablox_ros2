#include "voxblox_ros/transformer.h"

#include <minkindr/kindr_msg.h>
#include <minkindr/kindr_tf.h>

namespace voxblox {

Transformer::Transformer(const rclcpp::Node::SharedPtr& nh,
                         const rclcpp::Node::SharedPtr& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      sensor_frame_(""),
      timestamp_tolerance_ns_(1000000),
      tf_buffer_(),
      tf_listener_(tf_buffer_) {
  nh_private_->declare_parameter<std::string>("world_frame", world_frame_);
  nh_private_->get_parameter("world_frame", world_frame_);
  nh_private_->declare_parameter<std::string>("sensor_frame", sensor_frame_);
  nh_private_->get_parameter("sensor_frame", sensor_frame_);

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec = timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  nh_private_->declare_parameter<double>("timestamp_tolerance_sec", timestamp_tolerance_sec);
  nh_private_->get_parameter("timestamp_tolerance_sec", timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const rclcpp::Time& timestamp,
                                  Transformation* transform) {
  CHECK_NOTNULL(transform);
  return lookupTransformTf(from_frame, to_frame, timestamp, transform);
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const rclcpp::Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  tf2::StampedTransform tf_transform;
  rclcpp::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_listener_.canTransform(to_frame, from_frame_modified,
                                 time_to_lookup)) {
    return false;
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame_modified, time_to_lookup,
                                 tf_transform);
  } catch (tf2::TransformException& ex) {  // NOLINT
    RCLCPP_ERROR_STREAM(nh_private_->get_logger(),
                        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf2::transformTFToKindr(tf_transform, transform);
  return true;
}

}  // namespace voxblox
