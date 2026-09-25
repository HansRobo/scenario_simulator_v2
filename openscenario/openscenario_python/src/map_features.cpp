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

#include "map_features.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <limits>
#include <map>
#include <set>
#include <string>

namespace openscenario_python
{
namespace
{
const std::map<std::string, std::int8_t> BOUNDARY_TYPES = {
  {"crosswalk", 0},   {"curbstone", 1},          {"guard_rail", 2},  {"line_thick", 3},
  {"line_thin", 4},   {"pedestrian_marking", 5}, {"road_border", 6}, {"road_shoulder", 7},
  {"virtual", 8},     {"zebra_marking", 9}};
constexpr std::int8_t BOUNDARY_TYPE_VIRTUAL = 8;

const std::set<std::string> LANE_SUBTYPES = {
  "bicycle_lane", "crosswalk", "highway", "pedestrian_lane", "road", "road_shoulder", "walkway"};

template <typename LineString>
auto toPolyline(const LineString & line) -> Polyline
{
  Polyline points;
  points.reserve(line.size());
  for (const auto & point : line) {
    points.emplace_back(point.x(), point.y(), point.z());
  }
  return points;
}

auto arcLengths(const Polyline & input) -> std::vector<double>
{
  std::vector<double> s(input.size(), 0.0);
  for (std::size_t i = 1; i < input.size(); ++i) {
    s[i] = s[i - 1] + (input[i] - input[i - 1]).norm();
  }
  return s;
}

// Equal arc-length resampling that keeps the exact end points.
auto resample(const Polyline & input, std::size_t num_points) -> Polyline
{
  if (input.size() < 2 || num_points < 2) {
    return input;
  }
  const auto s = arcLengths(input);
  Polyline result;
  result.reserve(num_points);
  result.push_back(input.front());
  const double step = s.back() / static_cast<double>(num_points - 1);
  std::size_t segment = 0;
  for (std::size_t i = 1; i + 1 < num_points; ++i) {
    const double target = static_cast<double>(i) * step;
    while (segment + 1 < s.size() && s[segment + 1] < target) {
      ++segment;
    }
    segment = std::min(segment, s.size() - 2);
    const double length = std::max(s[segment + 1] - s[segment], 1e-6);
    const double t = std::clamp((target - s[segment]) / length, 0.0, 1.0);
    result.push_back(input[segment] + t * (input[segment + 1] - input[segment]));
  }
  result.push_back(input.back());
  return result;
}

// Akima spline over arc length, matching autoware_trajectory's interpolator including its
// boundary slopes and interval lookup, since road borders are resampled through it.
class Akima
{
public:
  Akima(const std::vector<double> & bases, const std::vector<double> & values) : bases_(bases)
  {
    const auto n = bases.size();
    std::vector<double> h(n - 1), m(n - 1), slope(n);
    for (std::size_t i = 0; i + 1 < n; ++i) {
      h[i] = bases[i + 1] - bases[i];
      m[i] = (values[i + 1] - values[i]) / h[i];
    }
    slope[0] = m[0];
    slope[1] = (m[0] + m[1]) / 2;
    for (std::size_t i = 2; i + 2 < n; ++i) {
      const double w0 = std::abs(m[i + 1] - m[i]);
      const double w1 = std::abs(m[i - 1] - m[i - 2]);
      slope[i] = (w0 + w1) == 0 ? (m[i] + m[i - 1]) / 2 : (w0 * m[i - 1] + w1 * m[i]) / (w0 + w1);
    }
    slope[n - 2] = (m[n - 2] + m[n - 3]) / 2;
    slope[n - 1] = m[n - 2];
    for (std::size_t i = 0; i + 1 < n; ++i) {
      a_.push_back(values[i]);
      b_.push_back(slope[i]);
      c_.push_back((3 * m[i] - 2 * slope[i] - slope[i + 1]) / h[i]);
      d_.push_back((slope[i] + slope[i + 1] - 2 * m[i]) / (h[i] * h[i]));
    }
  }

  auto operator()(double s) const -> double
  {
    s = std::clamp(s, bases_.front(), bases_.back());
    const auto i = index(s);
    const double dx = s - bases_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
  }

private:
  auto index(double s) const -> std::size_t
  {
    if (s == bases_.back()) {
      return bases_.size() - 2;
    }
    const auto it =
      std::lower_bound(bases_.begin(), bases_.end(), s, [](double a, double b) { return a <= b; });
    const auto distance = static_cast<std::size_t>(std::distance(bases_.begin(), it));
    return std::min(distance == 0 ? 0 : distance - 1, bases_.size() - 1);
  }

  std::vector<double> bases_, a_, b_, c_, d_;
};

// Splits a line string into as many num_points chunks as keep the point spacing within
// max_step_m, sampled through an Akima spline (linear below five points).
auto resampleRoadBorder(const Polyline & input, std::size_t num_points, double max_step_m)
  -> std::vector<Polyline>
{
  if (input.size() < 2 || num_points < 2) {
    return {input};
  }
  const auto s = arcLengths(input);
  const double total = s.back();
  constexpr double epsilon = 1e-6;
  if (total < epsilon) {
    return {Polyline(num_points, input.front())};
  }
  const double step = total / static_cast<double>(num_points - 1);
  const auto n_segments =
    static_cast<std::size_t>(std::max(1.0, std::ceil(step / std::max(max_step_m, epsilon))));

  std::function<Eigen::Vector3d(double)> at;
  if (input.size() >= 5) {
    std::vector<double> xs, ys, zs;
    for (const auto & p : input) {
      xs.push_back(p.x());
      ys.push_back(p.y());
      zs.push_back(p.z());
    }
    at = [x = Akima(s, xs), y = Akima(s, ys), z = Akima(s, zs)](double t) {
      return Eigen::Vector3d(x(t), y(t), z(t));
    };
  } else {
    at = [&input, &s](double t) {
      t = std::clamp(t, s.front(), s.back());
      std::size_t i = 0;
      while (i + 2 < s.size() && s[i + 1] < t) {
        ++i;
      }
      const double ratio = (t - s[i]) / (s[i + 1] - s[i]);
      return Eigen::Vector3d(input[i] + ratio * (input[i + 1] - input[i]));
    };
  }

  const double segment_length = total / static_cast<double>(n_segments);
  const double inner_step = segment_length / static_cast<double>(num_points - 1);
  std::vector<Polyline> result;
  for (std::size_t i = 0; i < n_segments; ++i) {
    Polyline points;
    for (std::size_t j = 0; j < num_points; ++j) {
      if (i == 0 && j == 0) {
        points.push_back(input.front());
      } else if (i + 1 == n_segments && j + 1 == num_points) {
        points.push_back(input.back());
      } else {
        const double t = static_cast<double>(i) * segment_length + static_cast<double>(j) * inner_step;
        points.push_back(at(std::clamp(t, 0.0, total)));
      }
    }
    result.push_back(std::move(points));
  }
  return result;
}

auto boundaryType(const lanelet::ConstLineString3d & bound) -> std::int8_t
{
  const auto it = BOUNDARY_TYPES.find(bound.attributeOr("type", ""));
  return it == BOUNDARY_TYPES.end() ? BOUNDARY_TYPE_VIRTUAL : it->second;
}

auto withinSquare(const Polyline & points, const Eigen::Vector3d & center, double range_m) -> bool
{
  return std::any_of(points.begin(), points.end(), [&](const Eigen::Vector3d & p) {
    return std::abs(p.x() - center.x()) <= range_m && std::abs(p.y() - center.y()) <= range_m;
  });
}
}  // namespace

MapFeatures::MapFeatures(const lanelet::LaneletMap & map, const MapFeatureConfig & config)
{
  for (const auto & lanelet : map.laneletLayer) {
    if (LANE_SUBTYPES.count(lanelet.attributeOr("subtype", "")) == 0) {
      continue;
    }
    LaneSegment lane;
    lane.id = lanelet.id();
    lane.centerline = resample(toPolyline(lanelet.centerline3d()), config.points_per_lane);
    lane.left_boundary = resample(toPolyline(lanelet.leftBound3d()), config.points_per_lane);
    lane.right_boundary = resample(toPolyline(lanelet.rightBound3d()), config.points_per_lane);
    lane.left_type = boundaryType(lanelet.leftBound3d());
    lane.right_type = boundaryType(lanelet.rightBound3d());
    lane.speed_limit_mps = std::numeric_limits<float>::quiet_NaN();
    if (lanelet.hasAttribute("speed_limit")) {
      lane.speed_limit_mps = std::stof(lanelet.attribute("speed_limit").value()) / 3.6f;
    }
    const std::string turn = lanelet.attributeOr("turn_direction", "");
    lane.turn_direction = turn == "straight" ? 0
                          : turn == "left"   ? 1
                          : turn == "right"  ? 2
                                             : LaneSegment::TURN_DIRECTION_NONE;
    // Older maps may carry more than one; the first is the one Autoware reads.
    const auto lights = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
    lane.traffic_light_id = lights.empty() ? LaneSegment::TRAFFIC_LIGHT_ID_NONE : lights.front()->id();
    lanes_.push_back(std::move(lane));
  }
  for (const auto & polygon : map.polygonLayer) {
    if (polygon.attributeOr("type", "") == std::string("intersection_area")) {
      intersection_areas_.push_back(
        resample(toPolyline(polygon.basicLineString()), config.points_per_intersection_area));
    }
  }
  for (const auto & line : map.lineStringLayer) {
    const std::string type = line.attributeOr("type", "");
    const auto points = toPolyline(line);
    if (type == "stop_line" && points.size() >= 2) {
      stop_lines_.push_back({points.front(), points.back()});
    } else if (type == "road_border") {
      for (auto & chunk :
           resampleRoadBorder(points, config.points_per_road_border, config.road_border_max_step_m)) {
        road_borders_.push_back(std::move(chunk));
      }
    }
  }
}

auto MapFeatures::nearestLanes(
  const Eigen::Isometry3d & map_to_ego, const Eigen::Vector3d & ego_position,
  std::size_t max_count, double range_m) const -> std::vector<std::size_t>
{
  const auto distance = [&](const Eigen::Vector3d & p) {
    const Eigen::Vector3d q = map_to_ego * p;
    const auto x = static_cast<float>(q.x());
    const auto y = static_cast<float>(q.y());
    return std::sqrt(x * x + y * y);
  };
  std::vector<std::pair<float, std::size_t>> candidates;
  for (std::size_t i = 0; i < lanes_.size(); ++i) {
    const auto & centerline = lanes_[i].centerline;
    if (centerline.size() < 2 || not withinSquare(centerline, ego_position, range_m)) {
      continue;
    }
    // The last point is the next lanelet's first, so it would tie adjacent lanelets.
    candidates.emplace_back(
      std::min(distance(centerline.front()), distance(centerline[centerline.size() - 2])), i);
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });
  std::vector<std::size_t> selected;
  for (std::size_t i = 0; i < candidates.size() && i < max_count; ++i) {
    selected.push_back(candidates[i].second);
  }
  return selected;
}

auto MapFeatures::routeLanes(
  const std::vector<std::int64_t> & route_lanelet_ids, const Eigen::Vector3d & ego_position,
  std::size_t max_count, double range_m) const -> std::vector<std::size_t>
{
  std::map<std::int64_t, std::size_t> index_of;
  for (std::size_t i = 0; i < lanes_.size(); ++i) {
    index_of[lanes_[i].id] = i;
  }
  std::vector<std::size_t> route;
  std::size_t closest = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (const auto id : route_lanelet_ids) {
    const auto it = index_of.find(id);
    if (it == index_of.end()) {
      continue;
    }
    route.push_back(it->second);
    for (const auto & p : lanes_[it->second].centerline) {
      if (const double d = (p - ego_position).norm(); d < closest_distance) {
        closest_distance = d;
        closest = route.size() - 1;
      }
    }
  }
  std::vector<std::size_t> selected;
  bool entered = false;
  for (std::size_t i = closest; i < route.size() && selected.size() < max_count; ++i) {
    if (not withinSquare(lanes_[route[i]].centerline, ego_position, range_m)) {
      if (entered) {
        break;
      }
      continue;
    }
    entered = true;
    selected.push_back(route[i]);
  }
  return selected;
}

auto MapFeatures::nearestPolylines(
  PolylineKind kind, const Eigen::Isometry3d & map_to_ego, const Eigen::Vector3d & ego_position,
  std::size_t max_count, double range_m) const -> std::vector<Polyline>
{
  const auto & source = kind == PolylineKind::intersection_area ? intersection_areas_
                        : kind == PolylineKind::stop_line       ? stop_lines_
                                                                : road_borders_;
  std::vector<std::pair<double, Polyline>> candidates;
  for (const auto & polyline : source) {
    const bool inside = std::any_of(polyline.begin(), polyline.end(), [&](const auto & p) {
      return std::abs(p.x() - ego_position.x()) < range_m &&
             std::abs(p.y() - ego_position.y()) < range_m;
    });
    if (not inside) {
      continue;
    }
    Polyline transformed;
    double nearest = std::numeric_limits<double>::max();
    for (const auto & p : polyline) {
      transformed.push_back(map_to_ego * p);
      nearest = std::min(nearest, std::hypot(transformed.back().x(), transformed.back().y()));
    }
    candidates.emplace_back(nearest, std::move(transformed));
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });
  std::vector<Polyline> selected;
  for (std::size_t i = 0; i < candidates.size() && i < max_count; ++i) {
    selected.push_back(std::move(candidates[i].second));
  }
  return selected;
}
}  // namespace openscenario_python
