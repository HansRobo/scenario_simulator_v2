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

namespace hdmap_utils
{
struct RoutingConfigurations
{
  bool allow_lane_change = false;
  bool use_road_shoulder = false;

  bool operator==(const RoutingConfigurations & routing_configuration) const
  {
    return allow_lane_change == routing_configuration.allow_lane_change &&
           use_road_shoulder == routing_configuration.use_road_shoulder;
  }

  struct HashFunction
  {
    size_t operator()(const RoutingConfigurations & c) const
    {
      return (std::hash<bool>()(c.allow_lane_change) << 1) ^ std::hash<bool>()(c.use_road_shoulder);
    }
  };

  friend std::ostream & operator<<(std::ostream & os, const RoutingConfigurations & rc)
  {
    os << "{allow_lane_change: " << (rc.allow_lane_change ? "true" : "false")
       << ", use_road_shoulder: " << (rc.use_road_shoulder ? "true" : "false") << "}";
    return os;
  }
};
}  // namespace hdmap_utils

namespace std
{
template <>
struct hash<hdmap_utils::RoutingConfigurations>
: public hdmap_utils::RoutingConfigurations::HashFunction
{
};
}  // namespace std

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__ROUTING_CONFIGURATIONS_HPP_
