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

#include <Eigen/Geometry>
#include <autoware/ml_planner/dimensions.hpp>
#include <autoware/ml_planner/preprocessing/preprocessing_utils.hpp>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <openscenario_interpreter/headless_bridge.hpp>
#include <rclcpp/time.hpp>
#include <stdexcept>

namespace openscenario_python
{
namespace bridge = openscenario_interpreter::headless;
namespace preprocess = autoware::ml_planner::preprocess;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using nav_msgs::msg::Odometry;

namespace
{
// The node's line_string_max_step_m, and the value ml_planner_data built the training data with.
constexpr double LINE_STRING_MAX_STEP_M = 5.0;
constexpr double TRAFFIC_LIGHT_TIMEOUT_S = 0.2;
constexpr std::uint8_t ENTITY_TYPE_PEDESTRIAN = 2;
// The preprocessing steps a history window back from the newest stamp, and rclcpp::Time cannot go
// below zero. Any constant shift leaves every interval unchanged.
constexpr double STAMP_OFFSET_S = 1000.0;

// Unique per entity name for the life of the process, which outlives any history.
auto uuidOf(const std::string & name) -> unique_identifier_msgs::msg::UUID
{
  unique_identifier_msgs::msg::UUID id;
  const std::size_t halves[] = {
    std::hash<std::string>{}(name), std::hash<std::string>{}(name + '\0')};
  std::memcpy(id.uuid.data(), halves, sizeof(halves));
  return id;
}

// Autoware tracks an object by its box centre; the simulator reports the entity's base_link.
auto boxCenter(const bridge::EntityState & state) -> geometry_msgs::msg::Pose
{
  const auto & q = state.pose.orientation;
  const auto & c = state.bounding_box.center;
  const Eigen::Vector3d offset = Eigen::Quaterniond(q.w, q.x, q.y, q.z) * Eigen::Vector3d(c.x, c.y, c.z);
  auto pose = state.pose;
  pose.position.x += offset.x();
  pose.position.y += offset.y();
  pose.position.z += offset.z();
  return pose;
}

auto mapOf() -> lanelet::LaneletMapConstPtr
{
  const auto map = bridge::laneletMap();
  if (not map) {
    throw std::runtime_error("no map is loaded; call activate() first");
  }
  return map;
}

auto vehicleSpecOf(const bridge::EntityState & ego) -> autoware::ml_planner::VehicleSpec
{
  const auto & box = ego.bounding_box;
  return {box.center.x + box.dimensions.x / 2, box.dimensions.x, box.dimensions.y};
}

// The goal lies on the route's last lanelet, at the height of its nearest centerline point: the goal
// enters the ego frame through a full 3D transform, so a missing height would shift it on a slope.
auto routeOf(
  const lanelet::LaneletMap & map, const std::vector<std::int64_t> & lanelet_ids,
  const std::vector<double> & goal_pose) -> autoware_planning_msgs::msg::LaneletRoute
{
  if (goal_pose.size() != 3 or lanelet_ids.empty()) {
    throw std::runtime_error("goal_pose must be (x, y, yaw) on a non-empty route");
  }
  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.frame_id = "map";
  route.goal_pose.position.x = goal_pose[0];
  route.goal_pose.position.y = goal_pose[1];
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto & point : map.laneletLayer.get(lanelet_ids.back()).centerline3d()) {
    const double distance = std::hypot(point.x() - goal_pose[0], point.y() - goal_pose[1]);
    if (distance < nearest) {
      nearest = distance;
      route.goal_pose.position.z = point.z();
    }
  }
  route.goal_pose.orientation.z = std::sin(goal_pose[2] / 2);
  route.goal_pose.orientation.w = std::cos(goal_pose[2] / 2);
  for (const auto id : lanelet_ids) {
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    segment.primitives.push_back(segment.preferred_primitive);
    route.segments.push_back(segment);
  }
  return route;
}
}  // namespace

MlPlannerObserver::MlPlannerObserver(
  const std::string & ego_ref, const std::vector<std::int64_t> & route_lanelet_ids,
  const std::vector<double> & goal_pose)
: ego_ref_(ego_ref),
  route_(routeOf(*mapOf(), route_lanelet_ids, goal_pose)),
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
  map_ = preprocess::build_map_context(mapOf(), LINE_STRING_MAX_STEP_M);
}

auto MlPlannerObserver::observe(std::uint8_t turn_indicator_report) -> void
{
  // Before its first frame the simulator reports no time; the state it holds then is time zero's.
  const double time = std::isnan(bridge::simulationTime()) ? 0.0 : bridge::simulationTime();
  // A ROS clock stamp: the node's buffers compare these with rclcpp::Time built from messages.
  const builtin_interfaces::msg::Time stamp =
    rclcpp::Time(std::llround((time + STAMP_OFFSET_S) * 1e9), RCL_ROS_TIME);

  TrackedObjects objects;
  objects.header.stamp = stamp;
  objects.header.frame_id = "map";
  for (const auto & state : bridge::entityStates()) {
    if (state.name == ego_ref_) {
      Odometry odometry;
      odometry.header.stamp = stamp;
      odometry.header.frame_id = "map";
      odometry.child_frame_id = "base_link";
      odometry.pose.pose = state.pose;
      odometry.twist.twist = state.twist;
      ego_.push_back(odometry);
      continue;
    }
    autoware_perception_msgs::msg::TrackedObject object;
    object.object_id = uuidOf(state.name);
    ObjectClassification classification;
    // The simulator's subtypes are ObjectClassification labels; a pedestrian may carry none.
    classification.label =
      state.type == ENTITY_TYPE_PEDESTRIAN ? ObjectClassification::PEDESTRIAN : state.subtype;
    classification.probability = 1.0f;
    object.classification.push_back(classification);
    object.kinematics.pose_with_covariance.pose = boxCenter(state);
    object.kinematics.twist_with_covariance.twist = state.twist;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions = state.bounding_box.dimensions;
    objects.objects.push_back(object);
  }
  objects_.push_back(objects);

  TurnIndicatorsReport report;
  report.stamp = stamp;
  report.report = turn_indicator_report;
  turn_indicators_.push_back(report);

  TrafficLightGroupArray lights;
  lights.stamp = stamp;
  for (const auto & group : bridge::conventionalTrafficLightGroups()) {
    autoware_perception_msgs::msg::TrafficLightGroup message;
    message.traffic_light_group_id = group.id;
    for (const auto & e : group.elements) {
      autoware_perception_msgs::msg::TrafficLightElement element;
      element.color = e.color;
      element.shape = e.shape;
      element.status = e.status;
      element.confidence = e.confidence;
      message.elements.push_back(element);
    }
    lights.traffic_light_groups.push_back(message);
  }
  traffic_lights_.push_back(lights);
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
