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

#ifndef OPENSCENARIO_PYTHON__ML_PLANNER_OBSERVER_HPP_
#define OPENSCENARIO_PYTHON__ML_PLANNER_OBSERVER_HPP_

// What autoware_ml_planner observes of the scenario, and the model inputs it builds from that.
// The messages are the ones the simulator would publish to Autoware, kept in the node's own
// history buffers; the inputs come from the node's own preprocessing, so a planner trained on
// Autoware logs is fed here exactly as it is on the vehicle.

#include <autoware/ml_planner/preprocessing/input_builder.hpp>
#include <autoware/ml_planner/utils/timed_buffer.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace openscenario_python
{
class MlPlannerObserver
{
public:
  // Requires an activated runner: the map and the ego exist then. goal_pose is (x, y, yaw).
  MlPlannerObserver(
    const std::string & ego_ref, const std::vector<std::int64_t> & route_lanelet_ids,
    const std::vector<double> & goal_pose);

  // Once per simulation step. turn_indicator_report is a TurnIndicatorsReport value.
  auto observe(std::uint8_t turn_indicator_report) -> void;

  // Every model input but initial_noise, normalized, without the batch axis.
  auto inputs() const -> autoware::ml_planner::preprocess::TensorMap;

private:
  template <typename T>
  using Buffer = autoware::ml_planner::utils::TimedBuffer<T>;

  std::string ego_ref_;
  std::unique_ptr<autoware::ml_planner::preprocess::LaneSegmentContext> map_;
  autoware_planning_msgs::msg::LaneletRoute route_;
  autoware::ml_planner::VehicleSpec vehicle_;
  Buffer<nav_msgs::msg::Odometry> ego_;
  Buffer<autoware_vehicle_msgs::msg::TurnIndicatorsReport> turn_indicators_;
  Buffer<autoware_perception_msgs::msg::TrackedObjects> objects_;
  Buffer<autoware_perception_msgs::msg::TrafficLightGroupArray> traffic_lights_;
};
}  // namespace openscenario_python

#endif  // OPENSCENARIO_PYTHON__ML_PLANNER_OBSERVER_HPP_
