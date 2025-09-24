// Copyright 2025 TIER IV, Inc.
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

#include "roi_based_detector/roi_based_detector_node.hpp"

#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/region_of_interest.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace roi_based_detector
{
void transformToRT(const geometry_msgs::msg::TransformStamped& tf, cv::Matx33d& R, cv::Vec3d& t)
{
  tf2::Quaternion q(
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z,
    tf.transform.rotation.w
  );

  tf2::Matrix3x3 m(q);

  R = cv::Matx33d(
    m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2]
  );

  t = cv::Vec3d(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf.transform.translation.z
  );
}

/**
 * @brief Compute the ground point of the bounding box in 3D space.
 * This function is assuming the bbox is 
 */
cv::Vec3d projectToGround(
  const cv::Point2f& pixel,
  const cv::Matx33d& K, const cv::Mat& D,
  const cv::Matx33d& R, const cv::Vec3d& t)
{
  std::vector<cv::Point2f> pixels = { pixel };
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(pixels, undistorted, K, D);

  cv::Vec3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
  cv::Vec3d ray_world = R * ray_cam;
  cv::Vec3d cam_origin = t;

  // get the ray length to the baselink's ground level (z=0)
  double lambda = -cam_origin[2] / ray_world[2];

  return cam_origin + lambda * ray_world;
}

/**
 * @brief Compute the height of the bounding box in 3D space.
 * This function use the top pixel of bbox to calculate the height by
 * casting rays to the near (x, y) point of bottom pixel.
 */
double computeHeight(
  const cv::Point2f& pixel_top,
  const cv::Matx33d& K, const cv::Mat& D,
  const cv::Matx33d& R, const cv::Vec3d& t,
  const cv::Vec3d& bottom_world)
{
  std::vector<cv::Point2f> pixels = { pixel_top };
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(pixels, undistorted, K, D);

  cv::Vec3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
  cv::Vec3d ray_world = R * ray_cam;
  cv::Vec3d cam_origin = t;

  // search the ray length that hit the x, y of the bottom point
  // NOTE: it might be better to solve with least squares or something.
  double lambda_x = (bottom_world[0] - cam_origin[0]) / ray_world[0];
  double lambda_y = (bottom_world[1] - cam_origin[1]) / ray_world[1];
  // averaging for robustness
  double lambda = (lambda_x + lambda_y) / 2.0;

  cv::Vec3d top_point = cam_origin + lambda * ray_world;

  return top_point[2];
}

// initialize constructor
RoiBasedDetectorNode::RoiBasedDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("roi_based_detector_node", node_options)
{
  target_frame_ = declare_parameter<std::string>("target_frame");
  label_settings_.UNKNOWN = declare_parameter<bool>("ignore_class.UNKNOWN");
  label_settings_.CAR = declare_parameter<bool>("ignore_class.CAR");
  label_settings_.TRUCK = declare_parameter<bool>("ignore_class.TRUCK");
  label_settings_.BUS = declare_parameter<bool>("ignore_class.BUS");
  label_settings_.TRAILER = declare_parameter<bool>("ignore_class.TRAILER");
  label_settings_.MOTORCYCLE = declare_parameter<bool>("ignore_class.MOTORCYCLE");
  label_settings_.BICYCLE = declare_parameter<bool>("ignore_class.BICYCLE");
  label_settings_.PEDESTRIAN = declare_parameter<bool>("ignore_class.PEDESTRIAN");

  std::vector<long> roi_ids = declare_parameter<std::vector<long>>("roi_ids");
  size_t rois_number = roi_ids.size();

  // create subscriber and publisher
  camera_info_subs_.resize(rois_number);
  roi_subs_.resize(rois_number);
  for (auto roi_id_index = 0u; roi_id_index < rois_number; ++roi_id_index) {
    const int roi_id = roi_ids[roi_id_index];
    const std::string roi_id_str = std::to_string(roi_id);

    // subscriber: camera info
    const std::string camera_info_topic_name = declare_parameter<std::string>(
      "input/rois" + roi_id_str + "/camera_info",
      "/sensing/camera/camera" + roi_id_str + "/camera_info");
    const auto camera_info_qos = rclcpp::QoS{1}.best_effort();

    camera_info_subs_[roi_id_index] = this->create_subscription<CameraInfo>(
      camera_info_topic_name, camera_info_qos, [this, roi_id](
        const CameraInfo::ConstSharedPtr msg) {
          this->cameraInfoCallback(msg, roi_id);
        }
      );

    // subscriber: roi
    const std::string roi_topic_name = declare_parameter<std::string>(
      "input/rois" + roi_id_str,
      "/perception/object_recognition/detection/rois" + roi_id_str);
    const auto roi_qos = rclcpp::QoS{1}.best_effort();

    roi_subs_[roi_id_index] = this->create_subscription<DetectedObjectsWithFeature>(
      roi_topic_name, roi_qos, [this, roi_id](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
        this->roiCallback(msg, roi_id);
      });

    // publisher
    const std::string output_topic_name = declare_parameter<std::string>(
      "output/rois" + roi_id_str + "/objects");
    objects_pubs_[roi_id] =
      this->create_publisher<DetectedObjects>(output_topic_name, 1);
  }

  transform_listener_ = std::make_shared<TransformListener>(this);
}

void RoiBasedDetectorNode::cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg, int roi_id)
{
  CameraInfo camera_info = *msg;
  camera_info_[roi_id] = camera_info;

  // assuming camera paramter never changes while running
  if (!is_inv_projection_initialized_[roi_id]) {
    Eigen::Matrix4f projection;
    projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
      camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
      camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
      camera_info.p.at(11), 0.0, 0.0, 0.0, 1.0;
    inv_projection_[roi_id] = projection.inverse();
    is_inv_projection_initialized_[roi_id] = true;

    // K is row-major 3x3
    cv::Matx33d K(
      camera_info.k[0], camera_info.k[1], camera_info.k[2],
      camera_info.k[3], camera_info.k[4], camera_info.k[5],
      camera_info.k[6], camera_info.k[7], camera_info.k[8]
    );

    cv::Mat D = cv::Mat(camera_info.d.size(), 1, CV_64F);
    for (size_t i = 0; i < camera_info.d.size(); i++) {
      D.at<double>(i, 0) = camera_info.d[i];
    }

    cam_intrinsics_[roi_id] = CameraIntrinsics{K, D};

    is_inv_projection_initialized_[roi_id] = true;
  }
}

/**
 * @brief Create a 3D object from a 2D ROI.
 * This function mainly target for pedestrian since the created object should not
 * be precise without the dephs information.
 */
void RoiBasedDetectorNode::createProjectedObject(
  const sensor_msgs::msg::RegionOfInterest & roi, const int & roi_id,
  const geometry_msgs::msg::TransformStamped & tf, const uint8_t & label,
  DetectedObject & object)
{
  CameraIntrinsics cam_intrinsics = cam_intrinsics_[roi_id];
  const cv::Matx33d K = cam_intrinsics.K;
  const cv::Mat D = cam_intrinsics.D;

  const uint32_t x_offset = roi.x_offset;
  const uint32_t y_offset = roi.y_offset;
  const uint32_t roi_width = roi.width;
  const uint32_t roi_height = roi.height;
  // use the ROI's bottom center for spawning position, and top center to approximate the height
  const cv::Point2f bottom_center(x_offset + roi_width * 0.5f, y_offset + roi_height);
  const cv::Point2f top_center(x_offset + roi_width * 0.5f, y_offset);
  // for deciding the diameter or width
  const cv::Point2f bottom_left(x_offset, y_offset + roi_height);
  const cv::Point2f bottom_right(x_offset + roi.width, y_offset + roi_height);

  cv::Matx33d R;
  cv::Vec3d t;
  transformToRT(tf, R, t);

  const cv::Vec3d bottom_point_in_3d = projectToGround(bottom_center, K, D, R, t);
  const double height = computeHeight(top_center, K, D, R, t, bottom_point_in_3d);

  const cv::Vec3d left_point_in_3d = projectToGround(bottom_left, K, D, R, t);
  const cv::Vec3d right_point_in_3d = projectToGround(bottom_right, K, D, R, t);

  const double dim_xy = cv::norm(left_point_in_3d-right_point_in_3d);

  geometry_msgs::msg::PoseStamped pose_stamped{};
  pose_stamped.pose.position.x = bottom_point_in_3d[0];
  pose_stamped.pose.position.y = bottom_point_in_3d[1];
  pose_stamped.pose.position.z = height * 0.5;
  object.kinematics.pose_with_covariance.pose = pose_stamped.pose;

  uint8_t shape_type = label_settings_.getLabelShape(label);
  if (
    shape_type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX ||
    shape_type == autoware_perception_msgs::msg::Shape::CYLINDER){
    object.shape.type = shape_type;
  } else {
    object.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;

    const float spacing = static_cast<float>(dim_xy);

    // add points for better visualization
    geometry_msgs::msg::Point32 p1;
    p1.x = -spacing * 0.5f;
    p1.y = -spacing * 0.5f;
    p1.z = 0.0f;
    object.shape.footprint.points.push_back(p1);

    geometry_msgs::msg::Point32 p2;
    p2.x = -spacing * 0.5f;
    p2.y = +spacing * 0.5f;
    p2.z = 0.0f;
    object.shape.footprint.points.push_back(p2);

    geometry_msgs::msg::Point32 p3;
    p3.x = +spacing * 0.5f;
    p3.y = +spacing * 0.5f;
    p3.z = 0.0f;
    object.shape.footprint.points.push_back(p3);

    geometry_msgs::msg::Point32 p4;
    p4.x = +spacing * 0.5f;
    p4.y = -spacing * 0.5f;
    p4.z = 0.0f;
    object.shape.footprint.points.push_back(p4);
  }

  object.shape.dimensions.x = dim_xy;
  object.shape.dimensions.y = dim_xy;
  object.shape.dimensions.z = height;
}

void RoiBasedDetectorNode::roiCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & msg, int roi_id)
{
  if (!is_inv_projection_initialized_[roi_id]) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info is not received yet");
    return;
  }

  DetectedObjects objects;

  // get transform from camera frame to base_link frame
  try {
    transform_ = transform_listener_->getTransform(
      target_frame_, msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to get transform: %s", ex.what());
    objects.header = msg->header;
    objects_pubs_[roi_id]->publish(objects);
    return;
  }

  if (!transform_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "getTransform failed. output objects will be empty.");
    std::cout << msg->header.frame_id << " to " << target_frame_ << std::endl;
    objects.header = msg->header;
    objects_pubs_[roi_id]->publish(objects);
    return;
  }

  if (!is_camera2lidar_mul_inv_projection_initialized_[roi_id]) {
    const Eigen::Matrix4f transform_matrix_cam2base =
      tf2::transformToEigen(transform_->transform).matrix().cast<float>();
    camera2lidar_mul_inv_projection_[roi_id] = transform_matrix_cam2base * inv_projection_[roi_id];
    is_camera2lidar_mul_inv_projection_initialized_[roi_id] = true;
  }

  for (const auto & obj_with_feature : msg->feature_objects) {
    DetectedObject object;

    const auto & label = obj_with_feature.object.classification.front().label;
    if (label_settings_.isIgnoreLabel(label)) {
      continue;
    }

    object.classification.push_back(obj_with_feature.object.classification.front());
    object.existence_probability = obj_with_feature.object.existence_probability;

    createProjectedObject(obj_with_feature.feature.roi, roi_id, *transform_, label, object);

    objects.objects.push_back(object);
  }

  objects.header = msg->header;
  objects.header.frame_id = target_frame_;
  objects_pubs_[roi_id]->publish(objects);
}

}  // namespace roi_based_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(roi_based_detector::RoiBasedDetectorNode)
