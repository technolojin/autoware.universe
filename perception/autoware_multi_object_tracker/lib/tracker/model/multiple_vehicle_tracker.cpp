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

#include "autoware/multi_object_tracker/tracker/model/multiple_vehicle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"

namespace autoware::multi_object_tracker
{
MultipleVehicleTracker::MultipleVehicleTracker(
  const rclcpp::Time & time, const types::DynamicObject & object)
: Tracker(time, object),
  normal_vehicle_tracker_(object_model::normal_vehicle, time, object),
  big_vehicle_tracker_(object_model::big_vehicle, time, object)
{
}

bool MultipleVehicleTracker::predict(const rclcpp::Time & time)
{
  big_vehicle_tracker_.predict(time);
  normal_vehicle_tracker_.predict(time);
  return true;
}

bool MultipleVehicleTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  big_vehicle_tracker_.measure(object, time, channel_info);
  normal_vehicle_tracker_.measure(object, time, channel_info);

  return true;
}

bool MultipleVehicleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object,
  [[maybe_unused]] const bool to_publish) const
{
  using Label = autoware_perception_msgs::msg::ObjectClassification;
  const uint8_t label = getHighestProbLabel();

  if (label == Label::CAR) {
    normal_vehicle_tracker_.getTrackedObject(time, object);
  } else if (label == Label::BUS || label == Label::TRUCK || label == Label::TRAILER) {
    big_vehicle_tracker_.getTrackedObject(time, object);
  }
  object.uuid = object_.uuid;
  object.classification = object_.classification;
  return true;
}

}  // namespace autoware::multi_object_tracker
