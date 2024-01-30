
#ifndef VISION_ROI_OBJECT_DETECTOR__NODE_HPP_
#define VISION_ROI_OBJECT_DETECTOR__NODE_HPP_

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace vision_roi_object_detector
{
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObject;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using sensor_msgs::msg::CameraInfo;
using autoware_auto_perception_msgs::msg::ObjectClassification;

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
  message_filters::Subscriber<CameraInfo>                 camera_info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    DetectedObjectsWithFeature, CameraInfo>
    SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  // Callback
  void objectsCallback(
    const DetectedObjectsWithFeature::ConstSharedPtr & input_roi_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  // Publisher
  std::string output_frame_id_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;
};

}  // namespace vision_roi_object_detector

#endif  // VISION_ROI_OBJECT_DETECTOR__NODE_HPP_
