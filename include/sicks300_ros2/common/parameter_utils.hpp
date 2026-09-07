// Copyright (c) 2022 Alberto J. Tudela Roldán
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

#ifndef SICKS300_ROS2__COMMON__PARAMETER_UTILS_HPP_
#define SICKS300_ROS2__COMMON__PARAMETER_UTILS_HPP_

#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/parameter_value.hpp"

namespace sicks300_ros2
{

/**
 * @brief Declares a ROS2 parameter and sets it to a given value if it was not already declared.
 *
 * @param node A node in which the given parameter is to be declared
 * @param param_name The name of the parameter
 * @param default_value Parameter value to initialize with
 * @param parameter_descriptor Parameter descriptor (optional)
 */
template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

}  // namespace sicks300_ros2

#endif  // SICKS300_ROS2__COMMON__PARAMETER_UTILS_HPP_
