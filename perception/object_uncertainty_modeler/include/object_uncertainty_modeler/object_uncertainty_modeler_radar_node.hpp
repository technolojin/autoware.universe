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

#ifndef OBJECT_UNCERTAINTY_MODELER__OBJECT_UNCERTAINTY_MODELER_RADAR_NODE_HPP_
#define OBJECT_UNCERTAINTY_MODELER__OBJECT_UNCERTAINTY_MODELER_RADAR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>

#include "radar_msgs/msg/radar_tracks.hpp"

#include <chrono>
#include <memory>
#include <vector>

namespace object_uncertainty_modeler
{
using radar_msgs::msg::RadarTrack;
using radar_msgs::msg::RadarTracks;

class ObjectUncertaintyModelerRadarNode : public rclcpp::Node
{
public:
  explicit ObjectUncertaintyModelerRadarNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    float uncertainty_hor_coeff[3] = {0.0, 0.0, 0.0};
    float uncertainty_long_coeff[3] = {0.0, 0.0, 0.0};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarTracks>::SharedPtr sub_tracks_{};

  // Callback
  void onTracks(const RadarTracks::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<RadarTracks>::SharedPtr pub_tracks_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  RadarTrack fillCovarianceMatrices(const RadarTrack & radar_track);
  Eigen::Matrix<float, 2, 2> calcPositionUncertainty(const geometry_msgs::msg::Point & position);
  Eigen::Matrix<float, 2, 2> calcVelocityUncertainty(const geometry_msgs::msg::Point & position, const geometry_msgs::msg::Vector3 & velocity);
};

}  // namespace object_uncertainty_modeler

#endif  // OBJECT_UNCERTAINTY_MODELER__OBJECT_UNCERTAINTY_MODELER_RADAR_NODE_HPP_
