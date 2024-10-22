#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <tf2_ros/transform_listener.h>

#include <rviz_common/frame_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rclcpp/logging.hpp>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay() {}

void VoxbloxMeshDisplay::onInitialize() {
  MFDClass::onInitialize();
}

VoxbloxMeshDisplay::~VoxbloxMeshDisplay() {}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMeshDisplay::processMessage(
    const voxblox_msgs::msg::Mesh::SharedPtr msg) {
  // Use the ROS 2 logging system
  rclcpp::Logger logger = rclcpp::get_logger("VoxbloxMeshDisplay");

  // Get the transform from the fixed frame to the frame in the message
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, rclcpp::Time(msg->header.stamp), position, orientation)) {
    RCLCPP_DEBUG(logger, "Error transforming from frame '%s' to frame '%s'",
                 msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Initialize visual if not already done
  if (visual_ == nullptr) {
    visual_ = std::make_unique<VoxbloxMeshVisual>(
        context_->getSceneManager(), scene_node_);
  }

  // Update the visual with the new message data
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay, rviz_common::MessageFilterDisplay<voxblox_msgs::msg::Mesh>)
