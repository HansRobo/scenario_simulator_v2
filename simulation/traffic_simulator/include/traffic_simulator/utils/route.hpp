// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__ROUTE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__ROUTE_HPP_

#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/utils/lanelet/lane_change.hpp>
#include <traffic_simulator/utils/lanelet/route.hpp>

namespace traffic_simulator
{
inline namespace route
{
template <typename... Ts>
auto getRoute(Ts &&... xs)
{
  return lanelet2::route::getRoute(std::forward<decltype(xs)>(xs)...);
}

auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) -> bool;

auto toSpline(const lanelet::Ids & route) -> math::geometry::CatmullRomSpline;

template <typename... Ts>
auto followingLanelets(Ts &&... xs)
{
  return lanelet2::route::getFollowingLanelets(std::forward<decltype(xs)>(xs)...);
}

template <typename... Ts>
auto previousLanelets(Ts &&... xs)
{
  return lanelet2::route::getPreviousLanelets(std::forward<decltype(xs)>(xs)...);
}

template <typename... Ts>
auto speedLimit(Ts &&... xs)
{
  return lanelet2::route::getSpeedLimit(std::forward<decltype(xs)>(xs)...);
}

auto isAnyConflictingEntity(
  const lanelet::Ids & following_lanelets,
  const std::vector<CanonicalizedLaneletPose> & other_poses) -> bool;

auto isNeedToRightOfWay(
  const lanelet::Ids & following_lanelets, std::vector<CanonicalizedLaneletPose> other_entity_poses)
  -> bool;

auto moveAlongLaneletPose(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Ids & route_lanelets,
  const double distance) -> traffic_simulator::LaneletPose;

auto moveBackPoints(const CanonicalizedLaneletPose & canonicalized_lanelet_pose)
  -> std::vector<geometry_msgs::msg::Point>;

template <typename... Ts>
auto laneChangeableLaneletId(Ts &&... xs)
{
  return lanelet2::lane_change::getLaneChangeableLaneletId(std::forward<decltype(xs)>(xs)...);
}

auto laneChangeAlongLaneletPose(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const traffic_simulator::lane_change::Parameter & parameter) -> traffic_simulator::LaneletPose;

auto laneChangeTrajectory(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const traffic_simulator::lane_change::Parameter & parameter)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

auto laneChangePoints(
  const math::geometry::HermiteCurve & curve, const double target_s, const double current_s,
  const double horizon, const traffic_simulator::lane_change::Parameter & parameter)
  -> std::vector<geometry_msgs::msg::Point>;
}  // namespace route
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__ROUTE_HPP_