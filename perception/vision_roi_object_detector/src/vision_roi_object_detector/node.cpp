// Copyright 2024 TIER IV, Inc.
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

#include "vision_roi_object_detector/node.hpp"

#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/ros/msg_covariance.hpp"

#include <memory>
#include <string>
#include <vector>
namespace vision_roi_object_detector
{

RoiObjectDetectorNode::RoiObjectDetectorNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("vision_roi_object_detector_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  rois_sub_(this, "input/rois", rclcpp::QoS{1}.reliable().get_rmw_qos_profile()),
  camera_info_sub_(this, "input/camera_info", rclcpp::QoS{1}.best_effort().get_rmw_qos_profile()),
  sync_(SyncPolicy(10), rois_sub_, camera_info_sub_)
{
  output_frame_id_ = declare_parameter<std::string>("output_frame_id");
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(std::bind(&RoiObjectDetectorNode::objectsCallback, this, _1, _2));

  // Publisher
  pub_objects_ = create_publisher<DetectedObjectsWithFeature>("~/output/roi_objects", rclcpp::QoS{1});
}

void RoiObjectDetectorNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & input_rois_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (pub_objects_->get_subscription_count() < 1) {
    return;
  }

  // 3D Object estimation from 2D ROI

  DetectedObjectsWithFeature output_objects;
  output_objects.header = input_rois_msg->header;
  output_objects.header.frame_id = output_frame_id_;

  // Camera information

  // camera height and pitch angle
  // obtain from camera frame id to base_link tf, wheel radius
  std::string camera_frame_id = camera_info_msg->header.frame_id;
  // get camera height and pitch angle from tf
  geometry_msgs::msg::TransformStamped camera_to_base_link_tf;
  try {
    camera_to_base_link_tf = tf_buffer_.lookupTransform(
      "base_link", camera_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }
  const double wheel_radius = 0.35; // [m]
  // camera height from the ground is calculated from the translation z coordinate of the tf and the wheel radius
  const double camera_height = camera_to_base_link_tf.transform.translation.z + wheel_radius; 
  // camera pitch angle is calculated from the rotation quaternion
  // while the euler angels in 3-2-1 rotation order
  const auto & camera_rotation_quaternion = camera_to_base_link_tf.transform.rotation;
  const double camera_pitch = std::asin(
    2.0 * (camera_rotation_quaternion.w * camera_rotation_quaternion.y -
    camera_rotation_quaternion.z * camera_rotation_quaternion.x));

  // camera focal length
  const double f_x = camera_info_msg->k[0];
  const double f_y = camera_info_msg->k[4];

  RCLCPP_INFO(get_logger(), "camera_pitch: %f, camera_height: %f", camera_pitch, camera_height);


  // for each feature object
  for (const auto & feature_object : input_rois_msg->feature_objects) {

    // Distance from camera to object
    // roi box bottom y coordinate
    auto & roi = feature_object.feature.roi;
    double roi_bottom = roi.y_offset + roi.height;
    // roi view angle
    double roi_view_angle = camera_pitch - std::atan2(roi_bottom, f_y);
    // if the view angle is above the horizon, the object is not detected
    if (roi_view_angle > 0) {
      // continue;
    }
    RCLCPP_INFO(get_logger(), "roi_bottom: %f, roi_view_angle: %f", roi_bottom, roi_view_angle);

    // object distance
    double distance = camera_height / std::tan(-roi_view_angle);
    RCLCPP_INFO(get_logger(), "distance: %f", distance);

    // Object azimuth angle
    // roi box center x coordinate
    double roi_center_x = roi.x_offset + roi.width / 2;

    // Object position in the camera frame
    geometry_msgs::msg::PointStamped estimated_object_position;
    estimated_object_position.header = input_rois_msg->header;
    estimated_object_position.header.frame_id = "base_link";
    estimated_object_position.point.x = distance;
    estimated_object_position.point.y = roi_center_x * distance / f_x;
    estimated_object_position.point.z = 0.0;
    
    // Object position in the base_link frame
    geometry_msgs::msg::PointStamped estimated_object_position_base_link;
    try {
      tf_buffer_.transform(
        estimated_object_position, estimated_object_position_base_link, "base_link");
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return;
    }


    // Object size
    // calculate object size based on roi box size on the image, distance, and camera focal length
    // The original object size is estimated by the object class

    // for test, the object size is set to default
    double object_size_x = 2.0;
    double object_size_y = 5.0;
    double object_size_z = 1.5;


    // Covariance matrix
    // when the roi has a certain uncertainty, the covariance matrix of the object is calculated
    // from the covariance matrix of the roi box and the distance

    // for test, the covariance matrix is set to default
    std::array<double, 36> object_pose_covariance{};
    using POSE_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

    object_pose_covariance[POSE_IDX::X_X] = 1.0; // x-x
    object_pose_covariance[POSE_IDX::Y_Y] = 1.0; // y-y
    object_pose_covariance[POSE_IDX::Z_Z] = 0.1; // z-z
    object_pose_covariance[POSE_IDX::ROLL_ROLL] = 0.1; // roll-roll
    object_pose_covariance[POSE_IDX::PITCH_PITCH] = 0.1; // pitch-pitch
    object_pose_covariance[POSE_IDX::YAW_YAW] = 3.14; // yaw-yaw


    // Fill in the output_objects
    DetectedObjectWithFeature output_object;
    output_object.feature = feature_object.feature;
    output_object.object.classification = feature_object.object.classification;

    output_object.object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    output_object.object.shape.dimensions.x = object_size_x;
    output_object.object.shape.dimensions.y = object_size_y;
    output_object.object.shape.dimensions.z = object_size_z;

    output_object.object.kinematics.pose_with_covariance.pose.position = estimated_object_position_base_link.point;
    output_object.object.kinematics.pose_with_covariance.covariance = object_pose_covariance;
    
    output_object.object.kinematics.has_twist = false;

    
    output_objects.feature_objects.push_back(output_object);

  }




  // debug message
  RCLCPP_INFO(get_logger(), "converted objects: %ld", output_objects.feature_objects.size());



  // output_objects.feature_objects = transformed_objects.feature_objects;

  pub_objects_->publish(output_objects);
}
}  // namespace vision_roi_object_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision_roi_object_detector::RoiObjectDetectorNode)
