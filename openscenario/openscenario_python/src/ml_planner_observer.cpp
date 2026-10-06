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

#include "ml_planner_observer.hpp"

#include <autoware/ml_planner/dimensions.hpp>
#include <autoware/ml_planner/preprocessing/preprocessing_utils.hpp>
#include <rclcpp/time.hpp>
#include <stdexcept>

#include "autoware_messages.hpp"

namespace openscenario_python
{
namespace bridge = openscenario_interpreter::headless;
namespace preprocess = autoware::ml_planner::preprocess;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using nav_msgs::msg::Odometry;

namespace
{
// The node's line_string_max_step_m, and the value ml_planner_data built the training data with.
constexpr double LINE_STRING_MAX_STEP_M = 5.0;
constexpr double TRAFFIC_LIGHT_TIMEOUT_S = 0.2;

auto vehicleSpecOf(const bridge::EntityState & ego) -> autoware::ml_planner::VehicleSpec
{
  const auto & box = ego.bounding_box;
  return {box.center.x + box.dimensions.x / 2, box.dimensions.x, box.dimensions.y};
}
}  // namespace

MlPlannerObserver::MlPlannerObserver(
  const std::string & ego_ref, const std::vector<std::int64_t> & route_lanelet_ids,
  const std::vector<double> & goal_pose)
: ego_ref_(ego_ref),
  route_(laneletRoute(*loadedMap(), route_lanelet_ids, goal_pose)),
  vehicle_(vehicleSpecOf(bridge::entityState(ego_ref))),
  ego_(autoware::ml_planner::HISTORY_WINDOW_S,
       [](const Odometry & m) { return rclcpp::Time(m.header.stamp); }),
  turn_indicators_(autoware::ml_planner::HISTORY_WINDOW_S,
                   [](const TurnIndicatorsReport & m) { return rclcpp::Time(m.stamp); }),
  objects_(autoware::ml_planner::HISTORY_WINDOW_S,
           [](const TrackedObjects & m) { return rclcpp::Time(m.header.stamp); }),
  traffic_lights_(autoware::ml_planner::HISTORY_WINDOW_S,
                  [](const TrafficLightGroupArray & m) { return rclcpp::Time(m.stamp); })
{
  map_ = preprocess::build_map_context(loadedMap(), LINE_STRING_MAX_STEP_M);
}

auto MlPlannerObserver::observe(std::uint8_t turn_indicator_report) -> void
{
  const auto messages = observeAutowareMessages(ego_ref_, turn_indicator_report);
  ego_.push_back(messages.odometry);
  objects_.push_back(messages.objects);
  turn_indicators_.push_back(messages.turn_indicators);
  traffic_lights_.push_back(messages.traffic_lights);
}

auto MlPlannerObserver::inputs() const -> preprocess::TensorMap
{
  if (ego_.empty()) {
    throw std::runtime_error("nothing observed yet; call observe() first");
  }
  const preprocess::FrameInputs frame{
    rclcpp::Time(ego_.back().header.stamp),
    preprocess::MessageView<Odometry>{ego_.msgs()},
    preprocess::MessageView<TurnIndicatorsReport>{turn_indicators_.msgs()},
    preprocess::MessageView<TrackedObjects>{objects_.msgs()},
    preprocess::MessageView<TrafficLightGroupArray>{traffic_lights_.msgs()},
    route_};
  auto result =
    preprocess::create_input_data_map(frame, *map_, vehicle_, {TRAFFIC_LIGHT_TIMEOUT_S});
  if (not result) {
    throw std::runtime_error(result.error());
  }
  preprocess::normalize_input_data(result->tensors);
  return std::move(result->tensors);
}
}  // namespace openscenario_python
