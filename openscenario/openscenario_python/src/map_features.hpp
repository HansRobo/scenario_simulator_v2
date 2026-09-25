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

#ifndef OPENSCENARIO_PYTHON__MAP_FEATURES_HPP_
#define OPENSCENARIO_PYTHON__MAP_FEATURES_HPP_

// The map half of a learned planner's observation: which lanes, route lanes and polylines surround
// the ego, resampled and moved into its frame. Which elements are picked and how they are
// resampled follows autoware_ml_planner's preprocessing, so a planner trained on Autoware logs sees
// the same geometry here; how they are packed into tensors is left to the caller.

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <lanelet2_core/LaneletMap.h>
#include <vector>

namespace openscenario_python
{
using Polyline = std::vector<Eigen::Vector3d>;

struct LaneSegment
{
  static constexpr std::int8_t TURN_DIRECTION_NONE = -1;
  static constexpr std::int64_t TRAFFIC_LIGHT_ID_NONE = -1;

  std::int64_t id;
  Polyline centerline;
  Polyline left_boundary;
  Polyline right_boundary;
  // Index into the boundary type table (crosswalk, curbstone, ..., virtual, zebra_marking).
  std::int8_t left_type;
  std::int8_t right_type;
  // NaN when the lanelet has no speed_limit attribute.
  float speed_limit_mps;
  std::int8_t turn_direction;  // -1 none, 0 straight, 1 left, 2 right
  std::int64_t traffic_light_id;
};

struct MapFeatureConfig
{
  std::size_t points_per_lane = 20;
  std::size_t points_per_intersection_area = 40;
  std::size_t points_per_road_border = 20;
  double road_border_max_step_m = 5.0;
};

class MapFeatures
{
public:
  MapFeatures(const lanelet::LaneletMap & map, const MapFeatureConfig & config);

  auto lane(std::size_t index) const -> const LaneSegment & { return lanes_[index]; }

  // Lanes with any centerline point in the square of half-width range_m around the ego, nearest
  // first by the ego-frame distance of their first or second-to-last centerline point.
  auto nearestLanes(
    const Eigen::Isometry3d & map_to_ego, const Eigen::Vector3d & ego_position,
    std::size_t max_count, double range_m) const -> std::vector<std::size_t>;

  // The route from the lanelet nearest the ego (in 3D) forward, stopping at the first lanelet that
  // leaves the square after having entered it.
  auto routeLanes(
    const std::vector<std::int64_t> & route_lanelet_ids, const Eigen::Vector3d & ego_position,
    std::size_t max_count, double range_m) const -> std::vector<std::size_t>;

  enum class PolylineKind { intersection_area, stop_line, road_border };

  // Ego-frame polylines with any point strictly inside the square, nearest point first.
  auto nearestPolylines(
    PolylineKind kind, const Eigen::Isometry3d & map_to_ego, const Eigen::Vector3d & ego_position,
    std::size_t max_count, double range_m) const -> std::vector<Polyline>;

private:
  std::vector<LaneSegment> lanes_;
  std::vector<Polyline> intersection_areas_;
  std::vector<Polyline> stop_lines_;
  std::vector<Polyline> road_borders_;
};
}  // namespace openscenario_python

#endif  // OPENSCENARIO_PYTHON__MAP_FEATURES_HPP_
