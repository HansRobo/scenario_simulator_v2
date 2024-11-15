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

#ifndef TRAFFIC_SIMULATOR__HDMAP_UTILS__ROUTING_CONFIGURATIONS_HPP_
#define TRAFFIC_SIMULATOR__HDMAP_UTILS__ROUTING_CONFIGURATIONS_HPP_
#include <iostream>
#include <traffic_simulator/data_type/routing_graph_type.hpp>

namespace hdmap_utils
{
struct RoutingConfigurations
{
  bool allow_lane_change = false;
  bool use_road_shoulder = false;
  traffic_simulator::RoutingGraphType routing_graph_type =
    traffic_simulator::RoutingGraphType::VEHICLE;

  bool operator==(const RoutingConfigurations & routing_configuration) const
  {
    return allow_lane_change == routing_configuration.allow_lane_change &&
           use_road_shoulder == routing_configuration.use_road_shoulder &&
           routing_graph_type == routing_configuration.routing_graph_type;
  }

  friend std::ostream & operator<<(std::ostream & os, const RoutingConfigurations & rc)
  {
    os << "{allow_lane_change: " << (rc.allow_lane_change ? "true" : "false")
       << ", use_road_shoulder: " << (rc.use_road_shoulder ? "true" : "false")
       << ", routing_graph_type: " << rc.routing_graph_type << "}";
    return os;
  }
};
}  // namespace hdmap_utils

namespace std
{
template <>
struct hash<hdmap_utils::RoutingConfigurations>
{
  std::size_t operator()(const hdmap_utils::RoutingConfigurations & config) const
  {
    auto h1 = std::hash<bool>{}(config.allow_lane_change);
    auto h2 = std::hash<bool>{}(config.use_road_shoulder);
    auto h3 = std::hash<std::uint8_t>{}(static_cast<std::uint8_t>(config.routing_graph_type));
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};
}  // namespace std

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__ROUTING_CONFIGURATIONS_HPP_
