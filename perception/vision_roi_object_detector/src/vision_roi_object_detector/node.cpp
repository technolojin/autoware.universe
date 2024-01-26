// Copyright 2023 TIER IV, Inc.
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
  pub_objects_ = create_publisher<DetectedObjectsWithFeature>("~/output/roi_object", rclcpp::QoS{1});
}

void RoiObjectDetectorNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & input_rois_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (pub_objects_->get_subscription_count() < 1) {
    return;
  }

  // DetectedObjectsWithFeature transformed_objects;
  // if (
  //   !object_recognition_utils::transformObjectsWithFeature(
  //     *input_rois_msg, output_frame_id_, tf_buffer_, transformed_objects)) {
  //   return;
  // }

  // camera info dummy 
  double num1 = camera_info_msg->k[0];
  double num2 = camera_info_msg->k[4];
  num1 = num1 + num2;

  // debug message
  RCLCPP_INFO(get_logger(), "num1: %f", num1);

  DetectedObjectsWithFeature output_objects;
  output_objects.header = input_rois_msg->header;


  // output_objects.feature_objects = transformed_objects.feature_objects;

  pub_objects_->publish(output_objects);
}
}  // namespace vision_roi_object_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision_roi_object_detector::RoiObjectDetectorNode)
