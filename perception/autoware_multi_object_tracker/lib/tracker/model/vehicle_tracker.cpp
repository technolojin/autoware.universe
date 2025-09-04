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

#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/tracker/model/vehicle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
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

#include <algorithm>

namespace autoware::multi_object_tracker
{
VehicleTracker::VehicleTracker(
  const object_model::ObjectModel & object_model, const rclcpp::Time & time,
  const types::DynamicObject & object)
: Tracker(time, object), logger_(rclcpp::get_logger("VehicleTracker")), object_model_(object_model)
{
  // set tracker type based on object model
  if (object_model.type == object_model::ObjectModelType::NormalVehicle) {
    tracker_type_ = TrackerType::NORMAL_VEHICLE;
  } else if (object_model.type == object_model::ObjectModelType::BigVehicle) {
    tracker_type_ = TrackerType::BIG_VEHICLE;
  } else if (object_model.type == object_model::ObjectModelType::Bicycle) {
    tracker_type_ = TrackerType::BICYCLE;
  } else {
    // not supported object model type
    RCLCPP_ERROR(logger_, "Unsupported object model type: %d", static_cast<int>(object_model.type));
    tracker_type_ = TrackerType::UNKNOWN;
  }

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
  motion_model_.setMotionParams(
    object_model_.process_noise, object_model_.bicycle_state, object_model_.process_limit);

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

    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_x_cov = object_model_.initial_covariance.vel_long;
    double vel_y_cov = object_model_.bicycle_state.init_slip_angle_cov;
    if (object.kinematics.has_twist) {
      vel_x = object.twist.linear.x;
      vel_y = object.twist.linear.y;
    }
    if (object.kinematics.has_twist_covariance) {
      vel_x_cov = object.twist_covariance[XYZRPY_COV_IDX::X_X];
      vel_y_cov = object.twist_covariance[XYZRPY_COV_IDX::Y_Y];
    }

    const double & length = object_.shape.dimensions.x;

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel_x, vel_x_cov, vel_y, vel_y_cov, length);
  }
}

bool VehicleTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool VehicleTracker::measureWithPose(
  const types::DynamicObject & object_in, const types::InputChannel & channel_info)
{
  types::DynamicObject object = object_in;
  // if the incoming object shape is polygon, convert it to bounding box
  if (object_in.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const double tracker_yaw = motion_model_.getYawState();
    if (shapes::convertConvexHullToBoundingBox(object_in, tracker_yaw, object)) {
      object.kinematics.orientation_availability = types::OrientationAvailability::AVAILABLE;
      object.shape.type =
        autoware_perception_msgs::msg::Shape::BOUNDING_BOX;  // not to fall into infinite loop
      object_.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;  // keep origin shape info
      measureWithPose(object, channel_info);
      return true;
    } else {
      // failed to convert, do not update
      return false;
    }
  }

  // get measurement yaw angle to update
  bool is_yaw_available =
    object.kinematics.orientation_availability != types::OrientationAvailability::UNAVAILABLE &&
    channel_info.trust_orientation;

  bool is_velocity_available = object.kinematics.has_twist;

  // check if the object is partially detected, and compensate the position
  bool is_long_partial_detect = false;
  bool is_close_to_front = true;
  {
    // project measured box to the tracker coordinate, yaw is already aligned
    types::DynamicObject tracker_object;
    getTrackedObject(getLatestMeasurementTime(), tracker_object, false);
    const double tracker_yaw = motion_model_.getYawState();
    const double cos_yaw = std::cos(tracker_yaw);
    const double sin_yaw = std::sin(tracker_yaw);
    const double dx = object.pose.position.x - tracker_object.pose.position.x;
    const double dy = object.pose.position.y - tracker_object.pose.position.y;
    const double local_x = dx * cos_yaw + dy * sin_yaw;
    const double local_y = -dx * sin_yaw + dy * cos_yaw;

    // pseudo intersection bounding box
    double intersect_left =
      std::min(tracker_object.shape.dimensions.y * 0.5, local_y - object.shape.dimensions.y * 0.5);
    double intersect_right =
      std::max(-tracker_object.shape.dimensions.y * 0.5, local_y - object.shape.dimensions.y * 0.5);
    double intersect_front = std::min(
      tracker_object.pose.position.x + tracker_object.shape.dimensions.x * 0.5,
      local_x + object.shape.dimensions.x * 0.5);
    double intersect_rear =
      std::max(-tracker_object.shape.dimensions.x * 0.5, local_x - object.shape.dimensions.x * 0.5);
    double intersect_length = std::max(0.0, intersect_front - intersect_rear);
    double intersect_width = std::max(0.0, intersect_left - intersect_right);
    double intersect_area = intersect_length * intersect_width;

    // precision to determine partial detection
    double source_area = tracker_object.shape.dimensions.x * tracker_object.shape.dimensions.y;
    double recall = source_area < 1e-6 ? 0.0 : intersect_area / source_area;

    // compensate detected object center position, if the object is partially detected
    // determine compensation direction and amount
    constexpr double min_recall_for_full_detection = 0.7;
    constexpr double min_length_ratio_for_partial_detection = 0.7;
    if (recall < min_recall_for_full_detection) {
      double comp_diff_x = 0.0;
      double comp_diff_y = 0.0;

      // if length is small, snap front or rear
      if (
        tracker_object.shape.dimensions.x * min_length_ratio_for_partial_detection >
        object.shape.dimensions.x) {
        is_long_partial_detect = true;

        // determine which part is close to the tracker bounding box
        double front_diff =
          local_x + (object.shape.dimensions.x - tracker_object.shape.dimensions.x) * 0.5;
        double rear_diff =
          local_x - (object.shape.dimensions.x - tracker_object.shape.dimensions.x) * 0.5;
        double front_rear_sum = std::abs(front_diff) + std::abs(rear_diff);
        double front_rear_weight =
          front_rear_sum < 1e-1 ? 0.5 : std::abs(rear_diff) / front_rear_sum;

        if (front_rear_weight > 0.5) {
          is_close_to_front = true;
          // snap front
          comp_diff_x = (tracker_object.shape.dimensions.x - object.shape.dimensions.x) * 0.5;
        } else {
          is_close_to_front = false;
          // snap rear
          comp_diff_x = -(tracker_object.shape.dimensions.x - object.shape.dimensions.x) * 0.5;
        }
      }

      // if width is small, snap left or right
      if (
        tracker_object.shape.dimensions.y * min_length_ratio_for_partial_detection >
        object.shape.dimensions.y) {
        double left_diff =
          local_y + (object.shape.dimensions.y - tracker_object.shape.dimensions.y) * 0.5;
        double right_diff =
          local_y - (object.shape.dimensions.y - tracker_object.shape.dimensions.y) * 0.5;
        double left_right_sum = std::abs(left_diff) + std::abs(right_diff);
        double left_right_weight =
          left_right_sum < 1e-1 ? 0.5 : std::abs(right_diff) / left_right_sum;

        if (left_right_weight > 0.5) {
          // snap left
          comp_diff_y = (tracker_object.shape.dimensions.y - object.shape.dimensions.y) * 0.5;
        } else {
          // snap right
          comp_diff_y = -(tracker_object.shape.dimensions.y - object.shape.dimensions.y) * 0.5;
        }
      }

      // compensate detected object position and footprint
      object.pose.position.x -= comp_diff_x * cos_yaw - comp_diff_y * sin_yaw;
      object.pose.position.y -= comp_diff_x * sin_yaw + comp_diff_y * cos_yaw;
      for (auto & p : object.shape.footprint.points) {
        p.x += comp_diff_x;
        p.y += comp_diff_y;
      }
    }
  }

  // update
  bool is_updated = false;
  {
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;
    const double yaw = tf2::getYaw(object.pose.orientation);
    const double vel_x = object.twist.linear.x;
    const double vel_y = object.twist.linear.y;
    constexpr double min_length = 1.0;  // minimum length to avoid division by zero
    const double length = std::max(object.shape.dimensions.x, min_length);

    if (is_long_partial_detect) {
      // update for partially detected object
      if (is_close_to_front) {
        double front_length = object.shape.dimensions.x * 0.5;
        double xf = x + front_length * std::cos(yaw) + front_length * std::sin(yaw);
        double yf = y - front_length * std::sin(yaw) + front_length * std::cos(yaw);
        is_updated = motion_model_.updateStatePoseFront(xf, yf, object.pose_covariance);
      } else {
        double rear_length = object.shape.dimensions.x * 0.5;
        double xr = x - rear_length * std::cos(yaw) - rear_length * std::sin(yaw);
        double yr = y + rear_length * std::sin(yaw) - rear_length * std::cos(yaw);
        is_updated = motion_model_.updateStatePoseRear(xr, yr, object.pose_covariance);
      }
    } else {
      if (is_yaw_available && is_velocity_available) {
        // update with yaw angle and velocity
        is_updated = motion_model_.updateStatePoseHeadVel(
          x, y, yaw, object.pose_covariance, vel_x, vel_y, object.twist_covariance, length);
      } else if (is_yaw_available && !is_velocity_available) {
        // update with yaw angle, but without velocity
        is_updated = motion_model_.updateStatePoseHead(x, y, yaw, object.pose_covariance, length);
      } else if (!is_yaw_available && is_velocity_available) {
        // update without yaw angle, but with velocity
        is_updated = motion_model_.updateStatePoseVel(
          x, y, object.pose_covariance, yaw, vel_x, vel_y, object.twist_covariance, length);
      } else {
        // update without yaw angle and velocity
        is_updated = motion_model_.updateStatePose(
          x, y, object.pose_covariance, length);  // update without yaw angle and velocity
      }
    }
  }

  // update motion model to limit states
  {
    bool is_flipped = false;
    motion_model_.limitStates(is_flipped);

    if (is_flipped) {
      // rotate footprint
      for (auto & p : object_.shape.footprint.points) {
        p.x = -p.x;
        p.y = -p.y;
      }
    }
  }

  // position z
  {
    constexpr double gain = 0.1;
    object_.pose.position.z =
      (1.0 - gain) * object_.pose.position.z + gain * object.pose.position.z;
  }

  // check object size abnormality
  constexpr double size_max_multiplier = 1.5;
  constexpr double size_min_multiplier = 0.25;
  if (
    object.shape.dimensions.x > object_model_.size_limit.length_max * size_max_multiplier ||
    object.shape.dimensions.x < object_model_.size_limit.length_min * size_min_multiplier ||
    object.shape.dimensions.y > object_model_.size_limit.width_max * size_max_multiplier ||
    object.shape.dimensions.y < object_model_.size_limit.width_min * size_min_multiplier) {
    return false;
  }

  // update object size
  {
    constexpr double gain = 0.4;
    constexpr double gain_inv = 1.0 - gain;
    auto & object_extension = object_.shape.dimensions;
    object_extension.x = motion_model_.getLength();  // tracked by motion model
    object_extension.y = gain_inv * object_extension.y + gain * object.shape.dimensions.y;
    object_extension.z = gain_inv * object_extension.z + gain * object.shape.dimensions.z;
  }

  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // set shape type, which is bounding box
  object_.area = types::getArea(object.shape);
  object_.shape.footprint = object.shape.footprint;

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

  // update object
  types::DynamicObject updating_object = in_object;
  // turn 180 deg if the updating object heads opposite direction
  {
    const double this_yaw = motion_model_.getYawState();
    const double updating_yaw = tf2::getYaw(updating_object.pose.orientation);
    double yaw_diff = updating_yaw - this_yaw;
    while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
    if (std::abs(yaw_diff) > M_PI_2) {
      tf2::Quaternion q;
      q.setRPY(0, 0, updating_yaw + M_PI);
      updating_object.pose.orientation = tf2::toMsg(q);
    }
  }

  // update pose
  measureWithPose(updating_object, channel_info);

  // remove cached object
  removeCache();

  return true;
}

bool VehicleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  // try to return cached object
  if (!getCachedObject(time, object)) {
    // if there is no cached object, predict and update cache
    object = object_;
    object.time = time;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

    // predict from motion model
    auto & pose = object.pose;
    auto & pose_cov = object.pose_covariance;
    auto & twist = object.twist;
    auto & twist_cov = object.twist_covariance;
    if (!motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov)) {
      RCLCPP_WARN(logger_, "VehicleTracker::getTrackedObject: Failed to get predicted state.");
      return false;
    }

    // cache object
    updateCache(object, time);
  }
  object.shape.dimensions.x = motion_model_.getLength();  // set length

  // if the tracker is to be published, check twist uncertainty
  // in case the twist uncertainty is large, lower the twist value
  if (to_publish) {
    using autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    // lower the x twist magnitude 1 sigma smaller
    // if the twist is smaller than 1 sigma, the twist is zeroed
    auto & twist = object.twist;
    constexpr double vel_cov_buffer = 0.7;  // [m/s] buffer not to limit certain twist
    constexpr double vel_too_low_ignore =
      0.25;  // [m/s] if the velocity is lower than this, do not limit
    const double vel_long = std::abs(twist.linear.x);
    if (vel_long > vel_too_low_ignore) {
      const double vel_limit = std::max(
        std::sqrt(object.twist_covariance[XYZRPY_COV_IDX::X_X]) - vel_cov_buffer, 0.0);  // [m/s]

      if (vel_long < vel_limit) {
        twist.linear.x = twist.linear.x > 0 ? vel_too_low_ignore : -vel_too_low_ignore;
      } else {
        double vel_suppressed = vel_long - vel_limit;
        vel_suppressed = std::max(vel_suppressed, vel_too_low_ignore);
        twist.linear.x = twist.linear.x > 0 ? vel_suppressed : -vel_suppressed;
      }
    }
  }

  return true;
}

}  // namespace autoware::multi_object_tracker
