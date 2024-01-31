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
  pub_objects_ = create_publisher<DetectedObjects>("~/output/roi_objects", rclcpp::QoS{1});

  // DEBUG roi box
  pub_debug_roi_ = create_publisher<DetectedObjects>("~/output/debug_roi", rclcpp::QoS{1});
}

void RoiObjectDetectorNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & input_rois_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (pub_objects_->get_subscription_count() < 1) {
    return;
  }

  // 3D Object estimation from 2D ROI
  DetectedObjects output_objects;
  output_objects.header = input_rois_msg->header;
  output_objects.header.frame_id = output_frame_id_;

  // DEBUG roi box
  DetectedObjects output_debug_rois;
  output_debug_rois.header = input_rois_msg->header;

  // Camera information

  // camera height and pitch angle
  // obtain from camera frame id to base_link tf
  std::string camera_frame_id = camera_info_msg->header.frame_id;
  // get camera height and pitch angle from tf
  geometry_msgs::msg::TransformStamped camera_to_base_link_tf;
  try {
    camera_to_base_link_tf =
      tf_buffer_.lookupTransform("base_link", camera_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }
  // camera height from the ground is calculated from the translation z coordinate of the tf
  geometry_msgs::msg::PointStamped camera_position;
  camera_position.header = camera_to_base_link_tf.header;
  camera_position.point.x = camera_to_base_link_tf.transform.translation.x;
  camera_position.point.y = camera_to_base_link_tf.transform.translation.y;
  camera_position.point.z = camera_to_base_link_tf.transform.translation.z;
  const double & camera_height = camera_position.point.z;

  // camera focal length
  const double c_x = camera_info_msg->k[2];
  const double c_y = camera_info_msg->k[5];
  const double f_x = camera_info_msg->k[0];
  const double f_y = camera_info_msg->k[4];

  // for each feature object
  for (const auto & feature_object : input_rois_msg->feature_objects) {

    // Object size
    // calculate object size based on roi box size on the image, distance, and camera focal length
    // The original object size is estimated by the object class

    // get highest probability classification
    const auto & classification =
      object_recognition_utils::getHighestProbClassification(feature_object.object.classification);
    using autoware_auto_perception_msgs::msg::ObjectClassification;

    // initial values
    geometry_msgs::msg::Vector3 object_size;

    // object size is estimated by the object class
    // the object size is based on typical vehicle size
    if (classification.label == ObjectClassification::PEDESTRIAN) {
      object_size.x = 0.5;
      object_size.y = 0.5;
      object_size.z = 1.5;

    } else if (classification.label == ObjectClassification::BICYCLE ||
               classification.label == ObjectClassification::MOTORCYCLE) {
      object_size.x = 1.8;
      object_size.y = 0.6;
      object_size.z = 1.5;
    } else if (classification.label == ObjectClassification::BUS ||
               classification.label == ObjectClassification::TRUCK ||
               classification.label == ObjectClassification::TRAILER) {
      object_size.x = 12.0;
      object_size.y = 2.5;
      object_size.z = 3.0;
    } else {
      object_size.x = 4.0;
      object_size.y = 1.8;
      object_size.z = 1.5;
    }


    // Distance from camera to object
    // roi box bottom y coordinate
    auto & roi = feature_object.feature.roi;
    double roi_x = roi.x_offset + roi.width / 2 - c_x;  // center x coordinate
    double roi_y = roi.y_offset + roi.height - c_y;     // bottom y coordinate

    // point ray vector
    // the vector from the camera center to the roi box bottom
    // the vector is normalized
    geometry_msgs::msg::PointStamped ray_vector_head;
    ray_vector_head.header = input_rois_msg->header;
    ray_vector_head.header.frame_id = camera_frame_id;
    ray_vector_head.point.x = roi_x / f_x;
    ray_vector_head.point.y = roi_y / f_y;
    ray_vector_head.point.z = 1.0;

    // ray vector is rotated by the camera tf
    geometry_msgs::msg::PointStamped ray_vector_head_base_link;
    try {
      tf_buffer_.transform(ray_vector_head, ray_vector_head_base_link, "base_link");
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return;
    }

    // calculate the ray vector
    geometry_msgs::msg::PointStamped ray_vector;
    ray_vector.point.x = ray_vector_head_base_link.point.x - camera_position.point.x;
    ray_vector.point.y = ray_vector_head_base_link.point.y - camera_position.point.y;
    ray_vector.point.z = ray_vector_head_base_link.point.z - camera_position.point.z;

    // if the ray vector is pointing over the horizon, the object is ignored
    if (ray_vector.point.z > 0) {
      continue;
    }

    // calculate ground point
    // the ground point is the intersection of the ray vector and the ground plane
    geometry_msgs::msg::PointStamped estimated_object_position;
    estimated_object_position.header = input_rois_msg->header;
    estimated_object_position.header.frame_id = "base_link";
    estimated_object_position.point.x =
      camera_position.point.x - camera_height / ray_vector.point.z * ray_vector.point.x;
    estimated_object_position.point.y =
      camera_position.point.y - camera_height / ray_vector.point.z * ray_vector.point.y;
    estimated_object_position.point.z = object_size.z / 2.0;

    // Object position in the publishing frame
    geometry_msgs::msg::PointStamped estimated_object_position_target_link;
    estimated_object_position_target_link.header.frame_id = output_frame_id_;

    try {
      tf_buffer_.transform(
        estimated_object_position, estimated_object_position_target_link, output_frame_id_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return;
    }


    // Covariance matrix
    // when the roi has a certain uncertainty, the covariance matrix of the object is calculated
    // from the covariance matrix of the roi box and the distance
    std::array<double, 36> object_pose_covariance{};
    using POSE_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

    // uncertainty in cylindrical coordinate system
    const double ray_vector_norm = std::sqrt(
      ray_vector.point.x * ray_vector.point.x + ray_vector.point.y * ray_vector.point.y +
      ray_vector.point.z * ray_vector.point.z);
    double distance = camera_height / (-ray_vector.point.z) * ray_vector_norm;
    double azimuth_angle = std::atan2(ray_vector.point.y, ray_vector.point.x);
    double vertical_angle = std::atan2(-ray_vector.point.z, std::sqrt(
      ray_vector.point.x * ray_vector.point.x + ray_vector.point.y * ray_vector.point.y));

    const double vertical_angle_uncertainty = 1.0 / 180.0 * M_PI;    // 1 degree
    const double horizontal_angle_uncertainty = 0.5 / 180.0 * M_PI;  // 0.5 degree
    const double vertical_roi_uncertainty = 3.0 / f_x; // 3 pixel
    const double horizontal_roi_uncertainty = 3.0 / f_y ; // 3 pixel

    const double distance_error = distance - camera_height / std::tan(vertical_angle + vertical_angle_uncertainty);
    const double horizontal_uncertainty = distance * (std::tan(horizontal_angle_uncertainty) + horizontal_roi_uncertainty) + 0.5;  // angular + roi uncertainty + size error
    const double longitudinal_uncertainty = distance_error + distance * vertical_roi_uncertainty + 1.0;   // angular distance uncertainty +  roi uncertainty + size error


    // rotate covariance vector by azimuth angle
    Eigen::Vector2d uncertainty_vector_radial(
      longitudinal_uncertainty * longitudinal_uncertainty,
      horizontal_uncertainty * horizontal_uncertainty);
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << std::cos(azimuth_angle), -std::sin(azimuth_angle),
      std::sin(azimuth_angle), std::cos(azimuth_angle);
    Eigen::Matrix2d horizontal_covariance_matrix =
      rotation_matrix * uncertainty_vector_radial.asDiagonal() * rotation_matrix.transpose();

    object_pose_covariance[POSE_IDX::X_X] = horizontal_covariance_matrix(0,0);          // x-x
    object_pose_covariance[POSE_IDX::X_Y] = horizontal_covariance_matrix(0,1);          // x-y
    object_pose_covariance[POSE_IDX::Y_X] = horizontal_covariance_matrix(1,0);          // y-x
    object_pose_covariance[POSE_IDX::Y_Y] = horizontal_covariance_matrix(1,1);          // y-y
    object_pose_covariance[POSE_IDX::Z_Z] = 0.1;          // z-z
    object_pose_covariance[POSE_IDX::ROLL_ROLL] = 0.1;    // roll-roll
    object_pose_covariance[POSE_IDX::PITCH_PITCH] = 0.1;  // pitch-pitch
    object_pose_covariance[POSE_IDX::YAW_YAW] = 3.14;     // yaw-yaw

    // Fill in the output_objects
    DetectedObject output_object;
    output_object.classification = feature_object.object.classification;

    output_object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    output_object.shape.dimensions = object_size;

    output_object.kinematics.has_position_covariance = true;
    output_object.kinematics.pose_with_covariance.pose.position =
      estimated_object_position_target_link.point;
    output_object.kinematics.pose_with_covariance.covariance = object_pose_covariance;

    output_object.kinematics.has_twist = false;
    output_object.kinematics.has_twist_covariance = false;
    output_object.kinematics.orientation_availability = false;

    output_objects.objects.push_back(output_object);

    // DEBUG roi box
    DetectedObject output_roi;
    output_roi.classification = feature_object.object.classification;
    output_roi.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
    const double roi_box_distance = 0.3;  // 30 cm front of the camera
    output_roi.shape.dimensions.x = roi.width / f_x * roi_box_distance;
    output_roi.shape.dimensions.y = roi.height / f_y * roi_box_distance;
    output_roi.shape.dimensions.z = 0.001;
    output_roi.kinematics.pose_with_covariance.pose.position.x =
      (roi.x_offset + roi.width / 2 - c_x) / f_x * roi_box_distance;
    output_roi.kinematics.pose_with_covariance.pose.position.y =
      (roi.y_offset + roi.height / 2 - c_y) / f_y * roi_box_distance;
    output_roi.kinematics.pose_with_covariance.pose.position.z = roi_box_distance;
    output_roi.kinematics.has_position_covariance = false;
    output_roi.kinematics.pose_with_covariance.covariance[0] = 0.1;
    output_debug_rois.objects.push_back(output_roi);
  }

  // debug message
  RCLCPP_INFO(get_logger(), "converted objects: %ld", output_objects.objects.size());

  pub_objects_->publish(output_objects);

  // DEBUG roi box
  pub_debug_roi_->publish(output_debug_rois);
}






}  // namespace vision_roi_object_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision_roi_object_detector::RoiObjectDetectorNode)
