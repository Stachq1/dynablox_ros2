#ifndef MINKINDR_CONVERSIONS_KINDR_MSG_H
#define MINKINDR_CONVERSIONS_KINDR_MSG_H

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/vector3.h>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

namespace tf2 {

// Function calls copied from eigen_conversions due to absence of the package in ROS 2
void vectorEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::msg::Vector3 &m) {
  m.x = e(0);
  m.y = e(1);
  m.z = e(2);
}

void vectorMsgToEigen(const geometry_msgs::msg::Vector3 &m, Eigen::Vector3d &e) {
  e(0) = m.x;
  e(1) = m.y;
  e(2) = m.z;
}

void quaternionEigenToMsg(const Eigen::Quaterniond &e, geometry_msgs::msg::Quaternion &m) {
  m.x = e.x();
  m.y = e.y();
  m.z = e.z();
  m.w = e.w();
}

void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion &m, Eigen::Quaterniond &e) {
  e = Eigen::Quaterniond(m.w, m.x, m.y, m.z);
}

// A wrapper for the relevant functions in eigen_conversions.
template <typename Scalar>
void quaternionKindrToMsg(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& kindr,
    geometry_msgs::msg::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr.toImplementation(), *msg);
}

// Also the Eigen implementation version of this.
template <typename Scalar>
void quaternionKindrToMsg(
    const Eigen::Quaternion<Scalar>& kindr, geometry_msgs::msg::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr, *msg);
}

template <typename Scalar>
void quaternionMsgToKindr(
    const geometry_msgs::Quaternion& msg,
    kindr::minimal::RotationQuaternionTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<Scalar> quat;
  quaternionMsgToEigen(msg, quat);
  *kindr = kindr::minimal::RotationQuaternionTemplate<Scalar>(quat);
}

template <typename Scalar>
void quaternionMsgToKindr(
    const geometry_msgs::Quaternion& msg, Eigen::Quaternion<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaternion<double> kindr_double;
  quaternionMsgToEigen(msg, kindr_double);
  *kindr = kindr_double.cast<Scalar>();
}

template <typename Scalar>
void vectorKindrToMsg(
    const Eigen::Matrix<Scalar, 3, 1>& kindr, geometry_msgs::msg::Vector3* msg) {
  CHECK_NOTNULL(msg);
  vectorEigenToMsg(kindr, *msg);
}

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::msg::Transform.
template <typename Scalar>
void transformKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& kindr,
    geometry_msgs::msg::Transform* msg) {
  CHECK_NOTNULL(msg);
  vectorKindrToMsg(kindr.getPosition(), &msg->translation);
  quaternionKindrToMsg(kindr.getRotation(), &msg->rotation);
}

template <typename Scalar>
void transformMsgToKindr(
    const geometry_msgs::msg::Transform& msg,
    kindr::minimal::QuatTransformationTemplate<Scalar>* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Matrix<Scalar, 3, 1> position;
  Eigen::Quaternion<Scalar> rotation;

  quaternionMsgToKindr(msg.rotation, &rotation);
  vectorMsgToKindr(msg.translation, &position);

  *kindr =
      kindr::minimal::QuatTransformationTemplate<Scalar>(rotation, position);
}

}  // namespace tf2

#endif  // MINKINDR_CONVERSIONS_KINDR_MSG_H
