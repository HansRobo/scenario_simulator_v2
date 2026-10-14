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
#include <cmath>
#include <rclcpp/time.hpp>
#include <stdexcept>

#include "autoware_messages.hpp"

namespace openscenario_python
{
namespace bridge = openscenario_interpreter::headless;
namespace preprocess = autoware::ml_planner::preprocess;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using nav_msgs::msg::Odometry;

namespace
{
#ifdef OPENSCENARIO_PYTHON_ML_PLANNER_MAP_STEP
// The node's line_string_max_step_m, and the value ml_planner_data built the training data with.
constexpr double LINE_STRING_MAX_STEP_M = 5.0;
#endif
constexpr double TRAFFIC_LIGHT_TIMEOUT_S = 0.2;

auto vehicleSpecOf(const bridge::EntityState & ego, const std::vector<double> & spec)
  -> autoware::ml_planner::VehicleSpec
{
#ifdef OPENSCENARIO_PYTHON_ML_PLANNER_STEERING
  // The simulator's vehicle parameters need not be the evaluated vehicle's, and the model reads
  // the wheel base, so it is not taken from the entity.
  if (spec.size() != 4) {
    throw std::invalid_argument(
      "vehicle_spec must be (base_link_to_front, length, width, wheel_base)");
  }
  return {spec[0], spec[1], spec[2], spec[3]};
#else
  if (not spec.empty()) {
    if (spec.size() < 3) {
      throw std::invalid_argument("vehicle_spec must be (base_link_to_front, length, width)");
    }
    return {spec[0], spec[1], spec[2]};
  }
  const auto & box = ego.bounding_box;
  return {box.center.x + box.dimensions.x / 2, box.dimensions.x, box.dimensions.y};
#endif
}
}  // namespace

MlPlannerObserver::MlPlannerObserver(
  const std::string & ego_ref, const std::vector<std::int64_t> & route_lanelet_ids,
  const std::vector<double> & goal_pose, const std::vector<double> & vehicle_spec)
: ego_ref_(ego_ref),
  route_(laneletRoute(*loadedMap(), route_lanelet_ids, goal_pose)),
  vehicle_(vehicleSpecOf(bridge::entityState(ego_ref), vehicle_spec)),
  ego_(autoware::ml_planner::HISTORY_WINDOW_S,
       [](const Odometry & m) { return rclcpp::Time(m.header.stamp); }),
  turn_indicators_(autoware::ml_planner::HISTORY_WINDOW_S,
                   [](const TurnIndicatorsReport & m) { return rclcpp::Time(m.stamp); }),
#ifdef OPENSCENARIO_PYTHON_ML_PLANNER_STEERING
  steering_(autoware::ml_planner::HISTORY_WINDOW_S,
            [](const SteeringReport & m) { return rclcpp::Time(m.stamp); }),
#endif
  objects_(autoware::ml_planner::HISTORY_WINDOW_S,
           [](const TrackedObjects & m) { return rclcpp::Time(m.header.stamp); }),
  traffic_lights_(autoware::ml_planner::HISTORY_WINDOW_S,
                  [](const TrafficLightGroupArray & m) { return rclcpp::Time(m.stamp); })
{
#ifdef OPENSCENARIO_PYTHON_ML_PLANNER_MAP_STEP
  map_ = preprocess::build_map_context(loadedMap(), LINE_STRING_MAX_STEP_M);
#else
  map_ = preprocess::build_map_context(loadedMap());
#endif
}

auto MlPlannerObserver::observe(std::uint8_t turn_indicator_report, double steering_tire_angle)
  -> void
{
  const auto messages = observeAutowareMessages(ego_ref_, turn_indicator_report);
  ego_.push_back(messages.odometry);
#ifdef OPENSCENARIO_PYTHON_ML_PLANNER_STEERING
  // An absent angle would read as driving straight, which the model cannot tell from the truth.
  if (not std::isfinite(steering_tire_angle)) {
    throw std::invalid_argument("this preprocessing reads steering; pass steering_tire_angle");
  }
  SteeringReport steering;
  steering.stamp = messages.odometry.header.stamp;
  steering.steering_tire_angle = static_cast<float>(steering_tire_angle);
  steering_.push_back(steering);
#else
  static_cast<void>(steering_tire_angle);
#endif
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
#ifdef OPENSCENARIO_PYTHON_ML_PLANNER_STEERING
    preprocess::MessageView<SteeringReport>{steering_.msgs()},
#endif
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
