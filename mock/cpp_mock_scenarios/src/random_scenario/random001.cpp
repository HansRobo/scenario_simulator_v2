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

#include <quaternion_operation/quaternion_operation.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <random001_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>

// headers in STL
#include <memory>
#include <random>
#include <string>
#include <vector>

class RandomScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit RandomScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "lanechange_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option),
    param_listener_(std::make_shared<random001::ParamListener>(get_node_parameters_interface())),
    engine_(seed_gen_())
  {
    start();
  }

private:
  std::shared_ptr<random001::ParamListener> param_listener_;
  random001::Params params_;
  std::random_device seed_gen_;
  std::mt19937 engine_;
  void spawnRoadParkingVehicles()
  {
    std::normal_distribution<> normal_dist(
      0.0, params_.random_parameters.road_parking_vehicle.s_variance);
    const auto spawn_road_parking_vehicle =
      [&](const auto & entity_index, const auto offset, const auto number_of_vehicles) {
        std::string entity_name = "road_parking_" + std::to_string(entity_index);
        constexpr lanelet::Id spawn_lanelet_id = 34705;
        api_.spawn(
          entity_name,
          api_.canonicalize(traffic_simulator::helper::constructLaneletPose(
            spawn_lanelet_id,
            static_cast<double>(entity_index) / static_cast<double>(number_of_vehicles) *
                api_.getLaneletLength(spawn_lanelet_id) +
              normal_dist(engine_),
            offset, 0, 0)),
          getVehicleParameters());
        api_.requestSpeedChange(entity_name, 0, true);
      };
    std::uniform_real_distribution<> dist(
      params_.random_parameters.road_parking_vehicle.min_offset,
      params_.random_parameters.road_parking_vehicle.max_offset);
    for (int i = 0; i < params_.random_parameters.road_parking_vehicle.number_of_vehicle; i++) {
      spawn_road_parking_vehicle(
        i, dist(engine_), params_.random_parameters.road_parking_vehicle.number_of_vehicle);
    }
  }

  void despawnRoadParkingVehicles()
  {
    for (int i = 0; i < params_.random_parameters.road_parking_vehicle.number_of_vehicle; i++) {
      api_.despawn("road_parking_" + std::to_string(i));
    }
  }

  void despawnCrossingPedestrians()
  {
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      std::string entity_name = "pedestrian" + std::to_string(i);
      if (api_.entityExists(entity_name)) {
        api_.despawn(entity_name);
      }
    }
  }

  void onUpdate() override
  {
    [&]() {
      if (param_listener_->is_old(params_)) {
        despawnRoadParkingVehicles();
        despawnCrossingPedestrians();
        param_listener_->refresh_dynamic_parameters();
        params_ = param_listener_->get_params();
        spawnRoadParkingVehicles();
      }
    }();

    const auto spawn_and_change_lane = [&](const auto & entity_name, const auto spawn_s_value) {
      api_.spawn(
        entity_name,
        api_.canonicalize(
          traffic_simulator::helper::constructLaneletPose(34513, spawn_s_value, 0, 0, 0, 0)),
        getVehicleParameters());
      api_.requestSpeedChange(entity_name, 10, true);
      api_.setLinearVelocity(entity_name, 10);
      api_.requestLaneChange(entity_name, traffic_simulator::lane_change::Direction::RIGHT);
    };

    if (api_.isInLanelet("ego", 34684, 0.1)) {
      if (!api_.entityExists("lane_following_0")) {
        spawn_and_change_lane("lane_following_0", 0.0);
      }
      if (!api_.entityExists("lane_following_1")) {
        spawn_and_change_lane("lane_following_1", 7.0);
      }
    }

    if (api_.isInLanelet("ego", 34606, 0.1)) {
      api_.requestAcquirePosition(
        "ego",
        api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34681, 0, 0, 0, 0, 0)));
    }
    if (api_.isInLanelet("ego", 34681, 0.1)) {
      api_.requestAcquirePosition(
        "ego",
        api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34606, 0, 0, 0, 0, 0)));
    }

    const auto spawn_and_cross_pedestrian = [&](const auto & entity_index) {
      std::string entity_name = "pedestrian" + std::to_string(entity_index);
      constexpr lanelet::Id lanelet_id = 34392;
      if (
        api_.entityExists(entity_name) &&
        std::abs(api_.getCurrentAccel(entity_name).linear.x) <= 0.01) {
        api_.despawn(entity_name);
      }
      if (
        !api_.entityExists(entity_name) &&
        !api_.reachPosition(
          "ego", api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34576, 25.0)),
          5.0)) {
        std::normal_distribution<> offset_distribution(
          0.0, params_.random_parameters.crossing_pedestrian.offset_variance);
        std::uniform_real_distribution<> speed_distribution(
          params_.random_parameters.crossing_pedestrian.min_speed,
          params_.random_parameters.crossing_pedestrian.max_speed);
        api_.spawn(
          entity_name,
          api_.canonicalize(traffic_simulator::helper::constructLaneletPose(
            lanelet_id, 0.0, offset_distribution(engine_))),
          getPedestrianParameters());
        const auto speed = speed_distribution(engine_);
        api_.requestSpeedChange(entity_name, speed, true);
        api_.setLinearVelocity(entity_name, speed);
      }
    };
    // spawn_and_cross_pedestrian("pedestrian_0", 34385);
    spawn_and_cross_pedestrian(0);
    for (int i = 0; i < params_.random_parameters.crossing_pedestrian.number_of_pedestrian; i++) {
      spawn_and_cross_pedestrian(i);
    }
  }
  void onInitialize() override
  {
    params_ = param_listener_->get_params();

    spawnRoadParkingVehicles();

    api_.spawn(
      "ego",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34621, 10, 0, 0, 0, 0)),
      getVehicleParameters());
    api_.requestAcquirePosition(
      "ego",
      api_.canonicalize(traffic_simulator::helper::constructLaneletPose(34606, 0, 0, 0, 0, 0)));
    api_.requestSpeedChange("ego", 10, true);
    api_.setLinearVelocity("ego", 10);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<RandomScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
