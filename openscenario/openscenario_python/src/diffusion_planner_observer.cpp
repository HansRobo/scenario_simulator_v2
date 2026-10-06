// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "diffusion_planner_observer.hpp"

#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>
#include <stdexcept>

#include "autoware_messages.hpp"

namespace openscenario_python
{
namespace bridge = openscenario_interpreter::headless;
namespace dp = autoware::diffusion_planner;

namespace
{
// The parameters the node's input building reads, at the node's defaults: one zero-temperature
// sample, no shift, delay or time interpolation.
auto nodeParams() -> dp::DiffusionPlannerParams
{
  dp::DiffusionPlannerParams params{};
  params.batch_size = 1;
  params.temperature_list = {0.0};
  params.traffic_light_group_msg_timeout_seconds = 0.2;
  params.line_string_max_step_m = 5.0;
  params.turn_indicator_hold_duration = 1.0;
  params.turn_indicator_keep_offset = -1.25f;
  return params;
}

// The ego's shape the way the node derives it from vehicle_info: the box spans the overhangs and
// the wheel base, and its centre is base_link_to_center ahead of base_link.
auto vehicleInfoOf(const bridge::EntityState & ego) -> autoware::vehicle_info_utils::VehicleInfo
{
  const auto & box = ego.bounding_box;
  const double length = box.dimensions.x;
  const double front_overhang = box.center.x + length / 2 - ego.wheel_base;
  const double rear_overhang = length / 2 - box.center.x;
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.0, 0.0, ego.wheel_base, box.dimensions.y, front_overhang, rear_overhang, 0.0, 0.0,
    box.dimensions.z, 0.0);
}
}  // namespace

DiffusionPlannerObserver::DiffusionPlannerObserver(
  const std::string & ego_ref, const std::vector<std::int64_t> & route_lanelet_ids,
  const std::vector<double> & goal_pose, const std::string & args_path)
: ego_ref_(ego_ref),
  route_(std::make_shared<const autoware_planning_msgs::msg::LaneletRoute>(
    laneletRoute(*loadedMap(), route_lanelet_ids, goal_pose))),
  normalization_(dp::utils::load_observation_normalization(args_path)),
  core_(std::make_unique<dp::DiffusionPlannerCore>(
    nodeParams(), vehicleInfoOf(bridge::entityState(ego_ref))))
{
  core_->set_map(loadedMap());
}

auto DiffusionPlannerObserver::observe(std::uint8_t turn_indicator_report) -> void
{
  const auto m = observeAutowareMessages(ego_ref_, turn_indicator_report);
  frame_ = core_->create_frame_context(
    std::make_shared<const nav_msgs::msg::Odometry>(m.odometry),
    std::make_shared<const geometry_msgs::msg::AccelWithCovarianceStamped>(m.acceleration),
    std::make_shared<const autoware_perception_msgs::msg::TrackedObjects>(m.objects),
    {std::make_shared<const autoware_perception_msgs::msg::TrafficLightGroupArray>(
      m.traffic_lights)},
    std::make_shared<const autoware_vehicle_msgs::msg::TurnIndicatorsReport>(m.turn_indicators),
    route_, rclcpp::Time(m.odometry.header.stamp));
}

auto DiffusionPlannerObserver::inputs() -> dp::InputDataMap
{
  if (not frame_) {
    throw std::runtime_error("nothing observed yet; call observe() first");
  }
  auto inputs = core_->create_input_data(*frame_);
  dp::preprocess::normalize_input_data(inputs, normalization_);
  return inputs;
}
}  // namespace openscenario_python
