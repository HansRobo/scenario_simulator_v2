/**
 * @file walk_straight_action.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class implimentation of the walk straight action
 * @version 0.1
 * @date 2021-04-02
 *
 * @copyright Copyright(c) Tier IV.Inc {2015-2021}
 *
 */

// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <simulation_api/behavior/pedestrian/walk_straight_action.hpp>

namespace entity_behavior
{
namespace pedestrian
{
WalkStraightAction::WalkStraightAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config) {}

void WalkStraightAction::getBlackBoardValues()
{
  PedestrianActionNode::getBlackBoardValues();
}

BT::NodeStatus WalkStraightAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status.lanelet_pose_valid) {
    setOutput(
      "updated_status",
      stopAtEndOfRoad());
    return BT::NodeStatus::RUNNING;
  }
  auto following_lanelets =
    hdmap_utils->getFollowingLanelets(entity_status.lanelet_pose.lanelet_id);
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
  }
  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.action_status.accel;

  double target_accel = (target_speed.get() - entity_status.action_status.twist.linear.x) /
    step_time;
  if (entity_status.action_status.twist.linear.x > target_speed.get()) {
    target_accel = boost::algorithm::clamp(target_accel, -5, 0);
    /*　target_accel = boost::algorithm::clamp(target_accel,
      -1*vehicle_param_ptr->performance.max_deceleration, vehicle_param_ptr->performance.max_acceleration);
      */
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, 3);
    /* target_accel = boost::algorithm::clamp(target_accel,
      -1*vehicle_param_ptr->performance.max_deceleration, vehicle_param_ptr->performance.max_acceleration);*/
  }
  accel_new.linear.x = target_accel;
  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = boost::algorithm::clamp(
    entity_status.action_status.twist.linear.x + accel_new.linear.x * step_time,
    0, 5.0);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;

  double new_s = entity_status.lanelet_pose.s +
    (twist_new.linear.x + entity_status.action_status.twist.linear.x) / 2.0 *
    step_time;
  geometry_msgs::msg::Vector3 rpy = entity_status.lanelet_pose.rpy;

  openscenario_msgs::msg::EntityStatus entity_status_updated;
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.lanelet_pose.lanelet_id = entity_status.lanelet_pose.lanelet_id;
  entity_status_updated.lanelet_pose.s = new_s;
  entity_status_updated.lanelet_pose.offset = entity_status.lanelet_pose.offset;
  entity_status_updated.lanelet_pose.rpy = rpy;
  entity_status_updated.action_status.twist = twist_new;
  entity_status_updated.action_status.accel = accel_new;
  entity_status_updated.pose = hdmap_utils->toMapPose(entity_status.lanelet_pose).pose;
  setOutput("updated_status", entity_status_updated);
  return BT::NodeStatus::RUNNING;
}
}      // namespace pedestrian
}  // namespace entity_behavior
