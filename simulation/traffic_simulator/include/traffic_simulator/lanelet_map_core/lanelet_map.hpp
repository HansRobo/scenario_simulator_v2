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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANELET_MAP_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANELET_MAP_HPP_

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace lanelet_map
{
using Point = geometry_msgs::msg::Point;
using Spline = math::geometry::CatmullRomSpline;

auto isInLanelet(const double s, const lanelet::Id lanelet_id) -> bool;

auto isInLanelet(const Point point, const lanelet::Id lanelet_id) -> bool;

auto laneletLength(const lanelet::Id lanelet_id) -> double;

auto laneletYaw(const Point & point, const lanelet::Id lanelet_id)
  -> std::tuple<double, Point, Point>;

auto laneletIds() -> lanelet::Ids;

auto nearbyLaneletIds(
  const Point &, const double distance_threshold, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids;

// Center points
auto centerPoints(const lanelet::Ids & lanelet_ids) -> std::vector<Point>;

auto centerPoints(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto centerPointsSpline(const lanelet::Id lanelet_id) -> std::shared_ptr<Spline>;

// Next lanelet
auto nextLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto nextLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids;

auto nextLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto nextLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids;

//Previous lanelet
auto previousLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto previousLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids;

auto previousLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto previousLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids;

//Bounds
auto leftBound(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto rightBound(const lanelet::Id lanelet_id) -> std::vector<Point>;

// Polygons
auto laneletPolygon(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto stopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<Point>;

// Relations
auto rightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>;

auto rightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto conflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto conflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

// private
auto toPolygon(const lanelet::ConstLineString3d & line_string) -> std::vector<Point>;

auto excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[])
  -> std::vector<std::pair<double, lanelet::Lanelet>>;
}  // namespace lanelet_map
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANELET_MAP_HPP_