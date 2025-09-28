/*
 * Copyright 2025 Fulvio Di Luzio
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rclcpp/node.hpp>
#include <string>
#include <vector>

/**
 * @brief Declare a bounded parameter for numeric types (int, float, double).
 *
 * @tparam T Numeric type (must be integral or floating-point).
 * @param[in] node Pointer to the ROS 2 node.
 * @param[in] name Base name of the parameter.
 * @param[in] default_value Default value if parameter is out of bounds
 * (optional).
 * @param[in] lower_bound Minimum allowed value (optional).
 * @param[in] upper_bound Maximum allowed value (optional).
 * @param[in] value Initial value (optional).
 *
 * @return true if bounds are valid and parameters declared, false otherwise.
 */
template <typename T>
std::enable_if_t<std::is_integral_v<T> || std::is_floating_point_v<T>, bool>
declareBoundedParameter(rclcpp::Node *node, std::string name,
                        std::optional<T> default_value = std::nullopt,
                        std::optional<T> lower_bound = std::nullopt,
                        std::optional<T> upper_bound = std::nullopt,
                        std::optional<T> value = std::nullopt) {

  T actual_default_value = default_value.value_or(static_cast<T>(0));
  T actual_lower_bound = lower_bound.value_or(std::numeric_limits<T>::min());
  T actual_upper_bound = upper_bound.value_or(std::numeric_limits<T>::max());
  T actual_value = value.value_or(static_cast<T>(0));

  if (lower_bound >= upper_bound) {
    RCLCPP_ERROR(node->get_logger(),
                 "Invalid bounds for parameter '%s': lower_bound (%.2f) "
                 "must be strictly less than upper_bound (%.2f).",
                 name.c_str(), static_cast<double>(actual_lower_bound),
                 static_cast<double>(actual_upper_bound));
    return false;
  }

  node->declare_parameter(name + ".default", actual_default_value);
  node->declare_parameter(name + ".lower_bound", actual_lower_bound);
  node->declare_parameter(name + ".upper_bound", actual_upper_bound);
  node->declare_parameter(name + ".value", actual_value);

  return true;
}

/**
 * @brief Declare a bounded parameter for string type with allowed values.
 *
 * @tparam T std::string.
 * @param[in] node Pointer to the ROS 2 node.
 * @param[in] name Base name of the parameter.
 * @param[in] default_value Default value if parameter is not in allowed values
 * (optional).
 * @param[in] allowed_values Vector of allowed string values (optional).
 * @param[in] value Initial value (optional).
 *
 * @return true if checks are passed, false otherwise.
 *
 * @note Creates three parameters: name.value, name.default, name.allowed_values
 */
template <typename T>
std::enable_if_t<std::is_same_v<T, std::string>, bool> declareBoundedParameter(
    rclcpp::Node *node, std::string name,
    std::optional<T> default_value = std::nullopt,
    std::optional<std::vector<T>> allowed_values = std::nullopt,
    std::optional<T> value = std::nullopt) {

  T actual_default_value = default_value.value_or("");
  std::vector<T> actual_allowed_values =
      allowed_values.value_or(std::vector<T>());
  T actual_value = value.value_or("");

  if (actual_allowed_values.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Allowed values for parameter '%s' cannot be empty.",
                 name.c_str());
    return false;
  }

  node->declare_parameter(name + ".default", actual_default_value);
  node->declare_parameter(name + ".allowed_values", actual_allowed_values);
  node->declare_parameter(name + ".value", actual_value);

  return true;
}

/**
 * @brief Get a bounded numeric parameter and validate against bounds.
 *
 * @tparam T Numeric type (must be integral or floating-point).
 * @param[in] node Pointer to the ROS 2 node.
 * @param[in] name Base name of the parameter.
 * @param[out] value Reference to store the parameter value.
 *
 * @return true If value is within bounds.
 * @return false If value is out of bounds (value is set to default).
 */
template <typename T>
std::enable_if_t<std::is_integral_v<T> || std::is_floating_point_v<T>, bool>
getBoundedParameter(rclcpp::Node *node, std::string name, T &value) {

  T lower_bound;
  T upper_bound;
  T default_value;

  node->get_parameter(name + ".value", value);
  node->get_parameter(name + ".lower_bound", lower_bound);
  node->get_parameter(name + ".upper_bound", upper_bound);
  node->get_parameter(name + ".default", default_value);

  if (value >= lower_bound && value <= upper_bound)
    return true;

  value = default_value;
  return false;
}

/**
 * @brief Get a bounded string parameter and validate against allowed values.
 *
 * @tparam T std::string.
 * @param[in] node Pointer to the ROS 2 node.
 * @param[in] name Base name of the parameter.
 * @param[out] value Reference to store the parameter value.
 *
 * @return true If value is in allowed_values list.
 * @return false If value is not allowed (value is set to default).
 */
template <typename T>
std::enable_if_t<std::is_same_v<T, std::string>, bool>
getBoundedParameter(rclcpp::Node *node, std::string name, T &value) {

  std::vector<T> allowed_values;
  T default_value;
  node->get_parameter(name + ".value", value);
  node->get_parameter(name + ".allowed_values", allowed_values);
  node->get_parameter(name + ".default", default_value);

  if (std::find(allowed_values.begin(), allowed_values.end(), value) !=
      allowed_values.end())
    return true;

  value = default_value;
  return false;
}
