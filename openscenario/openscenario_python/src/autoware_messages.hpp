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

#ifndef OPENSCENARIO_PYTHON__AUTOWARE_MESSAGES_HPP_
#define OPENSCENARIO_PYTHON__AUTOWARE_MESSAGES_HPP_

// The scenario as Autoware receives it: what the simulator would publish to a planner node at
// this instant, for the observers that feed a planner's own preprocessing.

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <cstdint>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <nav_msgs/msg/odometry.hpp>
#include <openscenario_interpreter/headless_bridge.hpp>
#include <string>
#include <vector>

namespace openscenario_python
{
struct AutowareMessages
{
  nav_msgs::msg::Odometry odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  autoware_perception_msgs::msg::TrackedObjects objects;
  autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_indicators;
  autoware_perception_msgs::msg::TrafficLightGroupArray traffic_lights;
};

// turn_indicator_report is a TurnIndicatorsReport value.
auto observeAutowareMessages(const std::string & ego_ref, std::uint8_t turn_indicator_report)
  -> AutowareMessages;

// Requires an activated runner.
auto loadedMap() -> lanelet::LaneletMapConstPtr;

// goal_pose is (x, y, yaw).
auto laneletRoute(
  const lanelet::LaneletMap & map, const std::vector<std::int64_t> & lanelet_ids,
  const std::vector<double> & goal_pose) -> autoware_planning_msgs::msg::LaneletRoute;
}  // namespace openscenario_python

#endif  // OPENSCENARIO_PYTHON__AUTOWARE_MESSAGES_HPP_
