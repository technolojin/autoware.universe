// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//
// Author: v1.0 Yukihiro Saito
//
#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/tracker/model/vehicle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/msg_covariance.hpp>

#include <bits/stdc++.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::multi_object_tracker
{
VehicleTracker::VehicleTracker(
  const object_model::ObjectModel & object_model, const rclcpp::Time & time,
  const types::DynamicObject & object)
: Tracker(time, object),
  object_model_(object_model),
  logger_(rclcpp::get_logger("VehicleTracker")),
  tracking_offset_(Eigen::Vector2d::Zero())
{
  // velocity deviation threshold
  //   if the predicted velocity is close to the observed velocity,
  //   the observed velocity is used as the measurement.
  velocity_deviation_threshold_ = autoware_utils::kmph2mps(10);  // [m/s]

  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // set default initial size
    auto & object_extension = object_.shape.dimensions;
    object_extension.x = object_model_.init_size.length;
    object_extension.y = object_model_.init_size.width;
    object_extension.z = object_model_.init_size.height;
  }
  object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // Set motion model parameters
  {
    const double q_stddev_acc_long = object_model_.process_noise.acc_long;
    const double q_stddev_acc_lat = object_model_.process_noise.acc_lat;
    const double q_stddev_yaw_rate_min = object_model_.process_noise.yaw_rate_min;
    const double q_stddev_yaw_rate_max = object_model_.process_noise.yaw_rate_max;
    const double q_stddev_slip_rate_min = object_model_.bicycle_state.slip_rate_stddev_min;
    const double q_stddev_slip_rate_max = object_model_.bicycle_state.slip_rate_stddev_max;
    const double q_max_slip_angle = object_model_.bicycle_state.slip_angle_max;
    const double lf_ratio = object_model_.bicycle_state.wheel_pos_ratio_front;
    const double lf_min = object_model_.bicycle_state.wheel_pos_front_min;
    const double lr_ratio = object_model_.bicycle_state.wheel_pos_ratio_rear;
    const double lr_min = object_model_.bicycle_state.wheel_pos_rear_min;
    motion_model_.setMotionParams(
      q_stddev_acc_long, q_stddev_acc_lat, q_stddev_yaw_rate_min, q_stddev_yaw_rate_max,
      q_stddev_slip_rate_min, q_stddev_slip_rate_max, q_max_slip_angle, lf_ratio, lf_min, lr_ratio,
      lr_min);
  }

  // Set motion limits
  {
    const double max_vel = object_model_.process_limit.vel_long_max;
    const double max_slip = object_model_.bicycle_state.slip_angle_max;
    motion_model_.setMotionLimits(max_vel, max_slip);  // maximum velocity and slip angle
  }

  // Set initial state
  {
    using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;
    const double yaw = tf2::getYaw(object.pose.orientation);

    auto pose_cov = object.pose_covariance;
    if (!object.kinematics.has_position_covariance) {
      // initial state covariance
      const auto & p0_cov_x = object_model_.initial_covariance.pos_x;
      const auto & p0_cov_y = object_model_.initial_covariance.pos_y;
      const auto & p0_cov_yaw = object_model_.initial_covariance.yaw;

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double sin_2yaw = std::sin(2.0 * yaw);
      pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
      pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
      pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
      pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];
      pose_cov[XYZRPY_COV_IDX::YAW_YAW] = p0_cov_yaw;
    }

    double vel = 0.0;
    double vel_cov = object_model_.initial_covariance.vel_long;
    if (object.kinematics.has_twist) {
      vel = object.twist.linear.x;
    }
    if (object.kinematics.has_twist_covariance) {
      vel_cov = object.twist_covariance[XYZRPY_COV_IDX::X_X];
    }

    const double slip = 0.0;
    const double slip_cov = object_model_.bicycle_state.init_slip_angle_cov;
    const double & length = object_.shape.dimensions.x;

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, slip, slip_cov, length);
  }
}

bool VehicleTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool VehicleTracker::update(
  const types::DynamicObject & object, const types::InputChannel & channel_info)
{
  types::DynamicObject in_object = object;

  // get matching point between self object and input object
  // get delta_x, delta_y, delta_yaw
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_yaw = 0.0;

  // anchor: most matching point between self object and input object
  // base on self object's coordinate
  double anchor_x = 0.0;
  double anchor_y = 0.0;
  {
    delta_x = in_object.pose.position.x - object_.pose.position.x;
    delta_y = in_object.pose.position.y - object_.pose.position.y;

    // size difference
    const double size_diff_x = in_object.shape.dimensions.x - object_.shape.dimensions.x;
    const double size_diff_y = in_object.shape.dimensions.y - object_.shape.dimensions.y;

    // yaw difference
    const double in_object_yaw = tf2::getYaw(in_object.pose.orientation);
    const double self_object_yaw = tf2::getYaw(object_.pose.orientation);
    double yaw_diff = in_object_yaw - self_object_yaw;
    while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

    // if objects are facing opposite direction, rotate input object 180 degrees
    if (yaw_diff > M_PI_2 || yaw_diff < -M_PI_2) {
      // rotate input object 180 degrees
      tf2::Quaternion q;
      q.setRPY(0, 0, in_object_yaw + M_PI);
      in_object.pose.orientation = tf2::toMsg(q);
      // rotate anchor point 180 degrees
      in_object.anchor_point.x = -in_object.anchor_point.x;
      in_object.anchor_point.y = -in_object.anchor_point.y;
    }

    // calculate anchor point
    if (in_object.anchor_point.x > 1e-6) {
      anchor_x = -size_diff_x / 2.0;
    } else if (in_object.anchor_point.x < -1e-6) {
      anchor_x = size_diff_x / 2.0;
    } else {
      anchor_x = 0;
    }
    if (in_object.anchor_point.y > 1e-6) {
      anchor_y = -size_diff_y / 2.0;
    } else if (in_object.anchor_point.y < -1e-6) {
      anchor_y = size_diff_y / 2.0;
    } else {
      anchor_y = 0;
    }

    // get yaw difference, but normalize to [-pi, pi]
    delta_yaw = tf2::getYaw(in_object.pose.orientation) - tf2::getYaw(object_.pose.orientation);
    while (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;
  }

  // get measurement yaw angle to update
  bool is_yaw_available =
    in_object.kinematics.orientation_availability != types::OrientationAvailability::UNAVAILABLE &&
    channel_info.trust_orientation;

  // velocity capability is checked only when the object has velocity measurement
  // and the predicted velocity is close to the observed velocity
  bool is_velocity_available = false;
  if (in_object.kinematics.has_twist) {
    const double tracked_vel = motion_model_.getStateElement(IDX::VEL);
    const double & observed_vel = in_object.twist.linear.x;
    if (std::fabs(tracked_vel - observed_vel) < velocity_deviation_threshold_) {
      // Velocity deviation is small
      is_velocity_available = true;
    }
  }

  // update motion model states
  bool is_updated = false;
  {
    const double x = object_.pose.position.x + delta_x;
    const double y = object_.pose.position.y + delta_y;
    const double yaw = tf2::getYaw(object_.pose.orientation) + delta_yaw;
    const double vel = in_object.twist.linear.x;

    if (is_yaw_available && is_velocity_available) {
      // update with yaw angle and velocity
      is_updated = motion_model_.updateStatePoseHeadVel(
        x, y, yaw, in_object.pose_covariance, vel, in_object.twist_covariance);
    } else if (is_yaw_available && !is_velocity_available) {
      // update with yaw angle, but without velocity
      is_updated = motion_model_.updateStatePoseHead(x, y, yaw, in_object.pose_covariance);
    } else if (!is_yaw_available && is_velocity_available) {
      // update without yaw angle, but with velocity
      is_updated = motion_model_.updateStatePoseVel(
        x, y, in_object.pose_covariance, vel, in_object.twist_covariance);
    } else {
      // update without yaw angle and velocity
      is_updated = motion_model_.updateStatePose(
        x, y, in_object.pose_covariance);  // update without yaw angle and velocity
    }
  }

  // position z
  constexpr double z_gain = 0.1;
  object_.pose.position.z =
    (1.0 - z_gain) * object_.pose.position.z + z_gain * in_object.pose.position.z;

  // update shape
  if (in_object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // do not update shape if the input is not a bounding box
    return false;
  }

  // check object size abnormality
  constexpr double size_max_multiplier = 1.5;
  constexpr double size_min_multiplier = 0.25;
  if (
    in_object.shape.dimensions.x > object_model_.size_limit.length_max * size_max_multiplier ||
    in_object.shape.dimensions.x < object_model_.size_limit.length_min * size_min_multiplier ||
    in_object.shape.dimensions.y > object_model_.size_limit.width_max * size_max_multiplier ||
    in_object.shape.dimensions.y < object_model_.size_limit.width_min * size_min_multiplier) {
    return false;
  }

  // update object size
  constexpr double size_gain = 0.1;
  constexpr double size_gain_inv = 1.0 - size_gain;
  auto & object_extension = object_.shape.dimensions;
  object_extension.x =
    size_gain_inv * object_extension.x + size_gain * in_object.shape.dimensions.x;
  object_extension.y =
    size_gain_inv * object_extension.y + size_gain * in_object.shape.dimensions.y;
  object_extension.z =
    size_gain_inv * object_extension.z + size_gain * in_object.shape.dimensions.z;

  // set shape type, which is bounding box
  object_.shape.type = in_object.shape.type;

  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // update motion model
  motion_model_.updateExtendedState(object_extension.x);

  // update offset into object position
  {
    // rotate back the offset vector from object coordinate to global coordinate
    const double yaw = motion_model_.getStateElement(IDX::YAW);
    const double offset_x_global = anchor_x * std::cos(yaw) - anchor_y * std::sin(yaw);
    const double offset_y_global = anchor_x * std::sin(yaw) + anchor_y * std::cos(yaw);
    motion_model_.adjustPosition(size_gain * offset_x_global, size_gain * offset_y_global);
  }

  // limit motion model states
  motion_model_.limitStates();

  return is_updated;
}

bool VehicleTracker::measure(
  const types::DynamicObject & in_object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "VehicleTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  // update self object from tracker states
  {
    object_.pose.position.x = motion_model_.getStateElement(IDX::X);
    object_.pose.position.y = motion_model_.getStateElement(IDX::Y);
    // convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, motion_model_.getStateElement(IDX::YAW));
    object_.pose.orientation = tf2::toMsg(q);
  }

  // update object
  update(in_object, channel_info);

  return true;
}

bool VehicleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object) const
{
  object = object_;
  object.time = time;

  // predict from motion model
  auto & pose = object.pose;
  auto & pose_cov = object.pose_covariance;
  auto & twist = object.twist;
  auto & twist_cov = object.twist_covariance;
  if (!motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov)) {
    RCLCPP_WARN(logger_, "VehicleTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // set shape
  const auto origin_yaw = tf2::getYaw(object_.pose.orientation);
  const auto ekf_pose_yaw = tf2::getYaw(pose.orientation);
  object.shape.footprint =
    autoware_utils::rotate_polygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);

  return true;
}

}  // namespace autoware::multi_object_tracker
