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

#ifndef OPENSCENARIO_PYTHON__DIFFUSION_PLANNER_OBSERVER_HPP_
#define OPENSCENARIO_PYTHON__DIFFUSION_PLANNER_OBSERVER_HPP_

// What autoware_diffusion_planner observes of the scenario, and the model inputs it builds from
// that. The node's core keeps its own histories and builds the inputs, so a Diffusion-Planner
// trained on data that core produced is fed here as it is on the vehicle.

#include <autoware/diffusion_planner/diffusion_planner_core.hpp>
#include <autoware/diffusion_planner/utils/arg_reader.hpp>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace openscenario_python
{
class DiffusionPlannerObserver
{
public:
  // Requires an activated runner. goal_pose is (x, y, yaw); args_path is the model's args.json,
  // which carries the observation normalization.
  DiffusionPlannerObserver(
    const std::string & ego_ref, const std::vector<std::int64_t> & route_lanelet_ids,
    const std::vector<double> & goal_pose, const std::string & args_path);

  // Once per simulation step. turn_indicator_report is a TurnIndicatorsReport value.
  auto observe(std::uint8_t turn_indicator_report) -> void;

  // Every model input the core builds, normalized, flattened, with a batch of one.
  auto inputs() -> autoware::diffusion_planner::InputDataMap;

private:
  std::string ego_ref_;
  std::shared_ptr<const autoware_planning_msgs::msg::LaneletRoute> route_;
  autoware::diffusion_planner::utils::ObservationNormalization normalization_;
  std::unique_ptr<autoware::diffusion_planner::DiffusionPlannerCore> core_;
  std::optional<autoware::diffusion_planner::FrameContext> frame_;
};
}  // namespace openscenario_python

#endif  // OPENSCENARIO_PYTHON__DIFFUSION_PLANNER_OBSERVER_HPP_
