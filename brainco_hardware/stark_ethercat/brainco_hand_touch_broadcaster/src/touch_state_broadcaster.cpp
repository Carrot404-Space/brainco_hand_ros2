// Copyright 2026 Joyson Robot
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

#include "brainco_hand_touch_broadcaster/touch_state_broadcaster.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace brainco_hand_touch_broadcaster
{

controller_interface::CallbackReturn TouchStateBroadcaster::on_init()
{
  auto_declare<std::string>("hand_name", "");
  auto_declare<std::string>("topic_name", "");
  auto_declare<std::vector<std::string>>("sensor_names", {});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TouchStateBroadcaster::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE,
    {}};
}

controller_interface::InterfaceConfiguration
TouchStateBroadcaster::state_interface_configuration() const
{
  if (!state_interface_names_.empty()) {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      state_interface_names_};
  }

  std::string hand_name;
  std::vector<std::string> sensor_names;
  if (get_node() != nullptr) {
    get_node()->get_parameter("hand_name", hand_name);
    get_node()->get_parameter("sensor_names", sensor_names);
  }

  if (!detail::is_valid_hand_name(hand_name)) {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {}};
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    detail::configured_state_interface_names(hand_name, sensor_names)};
}

controller_interface::CallbackReturn TouchStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  get_node()->get_parameter("hand_name", hand_name_);
  get_node()->get_parameter("topic_name", topic_name_);
  get_node()->get_parameter("sensor_names", sensor_names_);

  if (!detail::is_valid_hand_name(hand_name_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid hand_name '%s'. Expected 'left' or 'right'.",
      hand_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string error_message;
  if (!detail::validate_sensor_names(hand_name_, sensor_names_, &error_message)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error_message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (topic_name_.empty()) {
    topic_name_ = detail::default_topic_name(hand_name_);
  }

  state_interface_names_ = detail::expand_state_interface_names(sensor_names_);
  detail::initialize_message(message_, hand_name_, sensor_names_);
  publisher_ =
    get_node()->create_publisher<brainco_hand_touch_msgs::msg::TouchStateArray>(topic_name_, 10);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TouchStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (publisher_ == nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Touch state publisher was not created.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!has_required_state_interfaces()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Touch state broadcaster requires %zu assigned state interfaces but received %zu.",
      detail::kFingerCount * detail::kFieldCount,
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TouchStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (publisher_ != nullptr) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TouchStateBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publisher_.reset();
  message_ = brainco_hand_touch_msgs::msg::TouchStateArray{};
  state_interface_names_.clear();
  sensor_names_.clear();
  hand_name_.clear();
  topic_name_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TouchStateBroadcaster::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publisher_.reset();
  message_ = brainco_hand_touch_msgs::msg::TouchStateArray{};
  state_interface_names_.clear();
  sensor_names_.clear();
  hand_name_.clear();
  topic_name_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

detail::HandTouchValues TouchStateBroadcaster::read_touch_values() const
{
  detail::HandTouchValues touch_values{};

  for (std::size_t finger_index = 0; finger_index < detail::kFingerCount; ++finger_index) {
    for (std::size_t field_index = 0; field_index < detail::kFieldCount; ++field_index) {
      const std::size_t interface_index = finger_index * detail::kFieldCount + field_index;
      touch_values[finger_index][field_index] = state_interfaces_[interface_index].get_value();
    }
  }

  return touch_values;
}

bool TouchStateBroadcaster::has_required_state_interfaces() const
{
  if (
    state_interface_names_.empty() ||
    state_interface_names_.size() != detail::kFingerCount * detail::kFieldCount ||
    state_interfaces_.size() != state_interface_names_.size())
  {
    return false;
  }

  std::vector<std::string> actual_interface_names;
  actual_interface_names.reserve(state_interfaces_.size());
  for (std::size_t index = 0; index < state_interface_names_.size(); ++index) {
    actual_interface_names.push_back(state_interfaces_[index].get_name());
  }

  return detail::has_expected_interface_names(state_interface_names_, actual_interface_names);
}

controller_interface::return_type TouchStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (publisher_ == nullptr || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (!has_required_state_interfaces()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces but received %zu.",
      detail::kFingerCount * detail::kFieldCount,
      state_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  message_.header.stamp = static_cast<builtin_interfaces::msg::Time>(time);
  message_.hand_name = hand_name_;
  detail::assign_touch_values(message_, read_touch_values());
  publisher_->publish(message_);

  return controller_interface::return_type::OK;
}

}  // namespace brainco_hand_touch_broadcaster

PLUGINLIB_EXPORT_CLASS(
  brainco_hand_touch_broadcaster::TouchStateBroadcaster,
  controller_interface::ControllerInterface)
