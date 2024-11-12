// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_
#define TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_

#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/hdmap_utils/routing_configurations.hpp>
#include <unordered_map>
#include <vector>

namespace std
{
template <>
struct hash<std::tuple<lanelet::Id, lanelet::Id, hdmap_utils::RoutingConfigurations>>
{
public:
  size_t operator()(
    const std::tuple<lanelet::Id, lanelet::Id, hdmap_utils::RoutingConfigurations> & data) const
  {
    std::hash<lanelet::Id> lanelet_id_hash;
    std::hash<hdmap_utils::RoutingConfigurations> routing_configurations_hash;
    size_t seed = 0;
    // hash combine like boost library
    auto hash_combine = [](size_t & seed, size_t hash) {
      return seed ^ (hash + 0x9e3779b9 + (seed << 6) + (seed >> 2));
    };
    seed = hash_combine(seed, lanelet_id_hash(std::get<0>(data)));
    seed = hash_combine(seed, lanelet_id_hash(std::get<1>(data)));
    seed = hash_combine(seed, routing_configurations_hash(std::get<2>(data)));
    return seed;
  }
};
}  // namespace std

namespace hdmap_utils
{
class RouteCache
{
public:
  using KeyType = std::tuple<lanelet::Id, lanelet::Id, hdmap_utils::RoutingConfigurations>;
  auto exists(
    lanelet::Id from, lanelet::Id to,
    const hdmap_utils::RoutingConfigurations & routing_configurations) -> bool
  {
    std::lock_guard<std::mutex> lock(mutex_);
    KeyType key = {from, to, routing_configurations};
    return data_.find(key) != data_.end();
  }

  auto getRoute(
    const lanelet::Id from, const lanelet::Id to,
    const hdmap_utils::RoutingConfigurations & routing_configurations) -> decltype(auto)
  {
    if (!exists(from, to, routing_configurations)) {
      THROW_SIMULATION_ERROR(
        "route from : ", from, " to : ", to,
        (routing_configurations.allow_lane_change ? " with" : " without"),
        " lane change does not exists on route cache.");
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      KeyType key = {from, to, routing_configurations};
      return data_.at(key);
    }
  }

  auto appendData(
    lanelet::Id from, lanelet::Id to, const RoutingConfigurations & routing_configurations,
    const lanelet::Ids & route) -> void
  {
    std::lock_guard<std::mutex> lock(mutex_);
    KeyType key = {from, to, routing_configurations};
    data_[key] = route;
  }

private:
  std::unordered_map<KeyType, lanelet::Ids> data_;

  std::mutex mutex_;
};

class CenterPointsCache
{
public:
  auto exists(lanelet::Id lanelet_id) -> bool
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.find(lanelet_id) != data_.end();
  }

  auto getCenterPoints(lanelet::Id lanelet_id) -> decltype(auto)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.at(lanelet_id);
  }

  auto getCenterPointsSpline(lanelet::Id lanelet_id) -> decltype(auto)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return splines_[lanelet_id];
  }

  auto appendData(lanelet::Id lanelet_id, const std::vector<geometry_msgs::msg::Point> & route)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[lanelet_id] = route;
    splines_[lanelet_id] = std::make_shared<math::geometry::CatmullRomSpline>(route);
  }

private:
  std::unordered_map<lanelet::Id, std::vector<geometry_msgs::msg::Point>> data_;

  std::unordered_map<lanelet::Id, std::shared_ptr<math::geometry::CatmullRomSpline>> splines_;

  std::mutex mutex_;
};

class LaneletLengthCache
{
public:
  auto exists(lanelet::Id lanelet_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.find(lanelet_id) != data_.end();
  }

  auto getLength(lanelet::Id lanelet_id)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("length of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[lanelet_id];
  }

  auto appendData(lanelet::Id lanelet_id, double length)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[lanelet_id] = length;
  }

private:
  std::unordered_map<lanelet::Id, double> data_;

  std::mutex mutex_;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_
