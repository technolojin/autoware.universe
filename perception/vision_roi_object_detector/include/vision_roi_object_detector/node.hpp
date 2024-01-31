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

#ifndef VISION_ROI_OBJECT_DETECTOR__NODE_HPP_
#define VISION_ROI_OBJECT_DETECTOR__NODE_HPP_

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace vision_roi_object_detector
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

class RoiObjectDetectorNode : public rclcpp::Node
{
public:
  explicit RoiObjectDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  message_filters::Subscriber<DetectedObjectsWithFeature> rois_sub_;
  message_filters::Subscriber<CameraInfo> camera_info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<DetectedObjectsWithFeature, CameraInfo>
    SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  // Callback
  void objectsCallback(
    const DetectedObjectsWithFeature::ConstSharedPtr & input_roi_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  // Publisher
  std::string output_frame_id_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;

  // DEBUG roi box
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_debug_roi_;

  bool get_camera_position(
    const std::string & camera_frame_id, geometry_msgs::msg::Point & position);
  geometry_msgs::msg::Vector3 get_object_size(const DetectedObjectWithFeature & object);
  bool get_ray_vector(
    const geometry_msgs::msg::Point & camera_position,
    const geometry_msgs::msg::PointStamped & ray_head, geometry_msgs::msg::Vector3 & ray_vector);
  bool get_object_position(
    const geometry_msgs::msg::Point & camera_position,
    const geometry_msgs::msg::Vector3 & ray_vector, const geometry_msgs::msg::Vector3 & object_size,
    geometry_msgs::msg::Point & object_position);
};

}  // namespace vision_roi_object_detector

#endif  // VISION_ROI_OBJECT_DETECTOR__NODE_HPP_
