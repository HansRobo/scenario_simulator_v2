// Copyright 2024 Hakoroboken
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

#include <rclcpp/rclcpp.hpp>
// #include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/operator.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


struct quaternion
{
    double x;
    double y;
    double z;
    double w;
};

namespace oreore
{
namespace geometry
{
auto getRotationMatrix(quaternion quat) -> Eigen::Matrix3d
{
  auto x = quat.x;
  auto y = quat.y;
  auto z = quat.z;
  auto w = quat.w;
  Eigen::Matrix3d ret(3, 3);
  // clang-format off
  ret << x * x - y * y - z * z + w * w,  2 * (x * y - z * w),            2 * (z * x + w * y),
         2 * (x * y + z * w),           -x * x + y * y - z * z + w * w,  2 * (y * z - x * w), 
         2 * (z * x - w * y),            2 * (y * z + w * x),           -x * x - y * y + z * z + w * w;
  // clang-format on
  return ret;
}
}  // namespace geometry
}  // namespace math

namespace main_geometry
{
    class main_geometry_node : public rclcpp::Node
    {
    public:
        explicit main_geometry_node(const rclcpp::NodeOptions & node_options)
        : rclcpp::Node("main_geometry_node", node_options)
        {

            Eigen::Vector3d world_relative_position_(1.0, 2.0, 3.0);

            quaternion orientation;
            orientation.x = 0.0;
            orientation.y = 0.0;
            orientation.z = 0.0;
            orientation.w = 1.0;

            using math::geometry::operator*;
            const auto relative_position =
                math::geometry::getRotationMatrix(orientation) * world_relative_position_;

            std::cout << relative_position(0) 
                << ", " << relative_position(1)
                << ", " << relative_position(2) << std::endl;

            rclcpp::shutdown();
        }

    private:

    };
} // namespace main_geometry

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(main_geometry::main_geometry_node)