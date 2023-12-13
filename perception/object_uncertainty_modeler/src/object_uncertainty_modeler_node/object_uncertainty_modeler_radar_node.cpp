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

#include "object_uncertainty_modeler/object_uncertainty_modeler_radar_node.hpp"

#include "tier4_autoware_utils/ros/msg_covariance.hpp"

#include <memory>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace object_uncertainty_modeler
{
using radar_msgs::msg::RadarTrack;
using radar_msgs::msg::RadarTracks;

ObjectUncertaintyModelerRadarNode::ObjectUncertaintyModelerRadarNode(
  const rclcpp::NodeOptions & node_options)
: Node("object_uncertainty_modeler_radar", node_options)
{
  printf("ObjectUncertaintyModelerRadarNode::ObjectUncertaintyModelerRadarNode\n");

  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObjectUncertaintyModelerRadarNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.velocity_y_threshold = declare_parameter<double>("velocity_y_threshold", 7.0);

  // Subscriber
  sub_tracks_ = create_subscription<RadarTracks>(
    "~/input/tracks", rclcpp::QoS{1},
    std::bind(&ObjectUncertaintyModelerRadarNode::onTracks, this, std::placeholders::_1));

  // Publisher
  pub_tracks_ = create_publisher<RadarTracks>("~/output/tracks_mod", 1);
}

void ObjectUncertaintyModelerRadarNode::onTracks(const RadarTracks::ConstSharedPtr msg)
{
  // debug message
  printf("ObjectUncertaintyModelerRadarNode::onTracks\n");

  RadarTracks pub_tracks;

  pub_tracks.header = msg->header;

  for (const auto & radar_track : msg->tracks) {
    // fill covariance matrices
    auto pub_track = fillCovarianceMatrices(radar_track);

    pub_tracks.tracks.push_back(pub_track);
  }

  // publish
  pub_tracks_->publish(pub_tracks);
}

RadarTrack ObjectUncertaintyModelerRadarNode::fillCovarianceMatrices(const RadarTrack & radar_track)
{
  using RADAR_IDX = tier4_autoware_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;

  RadarTrack pub_track = radar_track;

  // calculate uncertainty
  const auto position = radar_track.position;
  float distance = std::sqrt(position.x * position.x + position.y * position.y);
  float horizontal_uncertainty_coeff = 0.01;    // [rad/m]
  float horizontal_uncertainty_base = 0.5;      // [m]
  float longitudinal_uncertainty_coeff = 0.02;  // [m/m]
  float longitudinal_uncertainty_base = 0.5;    // [m]

  // float azimuth = std::atan2(position.y, position.x);
  float horizontal_uncertainty =
    horizontal_uncertainty_coeff * distance * distance + horizontal_uncertainty_base;
  // float horizontal_uncertainty_sq = horizontal_uncertainty * horizontal_uncertainty;
  float longitudinal_uncertainty =
    longitudinal_uncertainty_coeff * distance + longitudinal_uncertainty_base;  // [m]
  // float longitudinal_uncertainty_sq = horizontal_uncertainty * horizontal_uncertainty;

  // rotate covariance matrix

  // fill covariance matrices
  auto & radar_position_cov = pub_track.position_covariance;
  radar_position_cov[RADAR_IDX::X_X] = longitudinal_uncertainty;
  radar_position_cov[RADAR_IDX::X_Y] = 0.0;
  radar_position_cov[RADAR_IDX::Y_Y] = horizontal_uncertainty;

  // const geometry_msgs::msg::Vector3 & vel = radar_track.velocity;

  auto & radar_vel_cov = pub_track.velocity_covariance;
  radar_vel_cov[RADAR_IDX::X_X] = 3.0;
  radar_vel_cov[RADAR_IDX::X_Y] = 0.0;
  radar_vel_cov[RADAR_IDX::Y_Y] = 5.0;

  auto & radar_accel_cov = pub_track.acceleration_covariance;
  radar_accel_cov[RADAR_IDX::X_X] = 3.0;
  radar_accel_cov[RADAR_IDX::X_Y] = 0.0;
  radar_accel_cov[RADAR_IDX::Y_Y] = 5.0;

  return pub_track;
}

rcl_interfaces::msg::SetParametersResult ObjectUncertaintyModelerRadarNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "velocity_y_threshold", p.velocity_y_threshold);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace object_uncertainty_modeler

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_uncertainty_modeler::ObjectUncertaintyModelerRadarNode)
