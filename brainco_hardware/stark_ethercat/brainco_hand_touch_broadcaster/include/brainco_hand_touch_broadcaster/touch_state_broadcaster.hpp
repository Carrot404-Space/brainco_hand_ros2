#ifndef BRAINCO_HAND_TOUCH_BROADCASTER__TOUCH_STATE_BROADCASTER_HPP_
#define BRAINCO_HAND_TOUCH_BROADCASTER__TOUCH_STATE_BROADCASTER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "brainco_hand_touch_msgs/msg/touch_state_array.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace brainco_hand_touch_broadcaster
{

struct TouchStateBroadcasterTestAccess;

namespace detail
{

inline constexpr std::size_t kFingerCount = 5;
inline constexpr std::size_t kFieldCount = 5;
inline const std::array<std::string, kFingerCount> kFingerNames = {
  "thumb", "index", "middle", "ring", "pinky"};
inline const std::array<std::string, kFieldCount> kTouchFields = {
  "normal_force", "tangential_force", "direction", "proximity", "status"};

using FingerTouchValues = std::array<double, kFieldCount>;
using HandTouchValues = std::array<FingerTouchValues, kFingerCount>;

inline bool is_valid_hand_name(const std::string & hand_name)
{
  return hand_name == "left" || hand_name == "right";
}

inline std::string default_topic_name(const std::string & hand_name)
{
  return "/" + hand_name + "_touch_states";
}

inline std::vector<std::string> expected_sensor_names(const std::string & hand_name)
{
  std::vector<std::string> sensor_names;
  sensor_names.reserve(kFingerCount);

  for (const auto & finger_name : kFingerNames) {
    sensor_names.emplace_back(hand_name + "_" + finger_name + "_touch");
  }

  return sensor_names;
}

inline bool validate_sensor_names(
  const std::string & hand_name,
  const std::vector<std::string> & sensor_names,
  std::string * error_message = nullptr)
{
  if (sensor_names.size() != kFingerCount) {
    if (error_message != nullptr) {
      *error_message = "sensor_names must contain exactly 5 entries";
    }
    return false;
  }

  const auto expected_names = expected_sensor_names(hand_name);
  for (std::size_t index = 0; index < expected_names.size(); ++index) {
    if (sensor_names[index] != expected_names[index]) {
      if (error_message != nullptr) {
        *error_message =
          "sensor_names must match expected order, missing " + expected_names[index];
      }
      return false;
    }
  }

  if (error_message != nullptr) {
    error_message->clear();
  }

  return true;
}

inline std::vector<std::string> expand_state_interface_names(
  const std::vector<std::string> & sensor_names)
{
  std::vector<std::string> interface_names;
  interface_names.reserve(sensor_names.size() * kTouchFields.size());

  for (const auto & sensor_name : sensor_names) {
    for (const auto & field_name : kTouchFields) {
      interface_names.emplace_back(sensor_name + "/" + field_name);
    }
  }

  return interface_names;
}

inline std::vector<std::string> configured_state_interface_names(
  const std::string & hand_name, const std::vector<std::string> & sensor_names)
{
  std::string error_message;
  if (!is_valid_hand_name(hand_name) || !validate_sensor_names(hand_name, sensor_names, &error_message))
  {
    return {};
  }

  return expand_state_interface_names(sensor_names);
}

inline bool has_expected_interface_names(
  const std::vector<std::string> & expected_names,
  const std::vector<std::string> & actual_names)
{
  return expected_names == actual_names;
}

inline std::string derive_finger_name(
  const std::string & sensor_name, const std::string & hand_name)
{
  const std::string prefix = hand_name + "_";
  const std::string suffix = "_touch";

  if (sensor_name.rfind(prefix, 0) != 0 || sensor_name.size() <= prefix.size() + suffix.size()) {
    return sensor_name;
  }

  return sensor_name.substr(
    prefix.size(), sensor_name.size() - prefix.size() - suffix.size());
}

inline std::uint32_t safe_double_to_uint32(double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0u;
  }

  constexpr double max_value = static_cast<double>(std::numeric_limits<std::uint32_t>::max());
  if (value >= max_value) {
    return std::numeric_limits<std::uint32_t>::max();
  }

  return static_cast<std::uint32_t>(value);
}

inline std::uint16_t safe_double_to_uint16(double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0u;
  }

  constexpr double max_value = static_cast<double>(std::numeric_limits<std::uint16_t>::max());
  if (value >= max_value) {
    return std::numeric_limits<std::uint16_t>::max();
  }

  return static_cast<std::uint16_t>(value);
}

inline void initialize_message(
  brainco_hand_touch_msgs::msg::TouchStateArray & message,
  const std::string & hand_name,
  const std::vector<std::string> & sensor_names)
{
  message.hand_name = hand_name;
  message.header.frame_id.clear();
  message.fingers.resize(sensor_names.size());

  for (std::size_t index = 0; index < sensor_names.size(); ++index) {
    message.fingers[index].finger_name = derive_finger_name(sensor_names[index], hand_name);
  }
}

inline void assign_touch_values(
  brainco_hand_touch_msgs::msg::TouchStateArray & message,
  const HandTouchValues & touch_values)
{
  for (std::size_t finger_index = 0; finger_index < message.fingers.size(); ++finger_index) {
    auto & finger_state = message.fingers[finger_index];
    const auto & values = touch_values[finger_index];

    finger_state.normal_force = static_cast<float>(values[0]);
    finger_state.tangential_force = static_cast<float>(values[1]);
    finger_state.direction = static_cast<float>(values[2]);
    finger_state.proximity = safe_double_to_uint32(values[3]);
    finger_state.status = safe_double_to_uint16(values[4]);
  }
}

}  // namespace detail

class TouchStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  friend struct TouchStateBroadcasterTestAccess;

  detail::HandTouchValues read_touch_values() const;
  bool has_required_state_interfaces() const;

  std::string hand_name_;
  std::string topic_name_;
  std::vector<std::string> sensor_names_;
  std::vector<std::string> state_interface_names_;
  rclcpp_lifecycle::LifecyclePublisher<brainco_hand_touch_msgs::msg::TouchStateArray>::SharedPtr
    publisher_;
  brainco_hand_touch_msgs::msg::TouchStateArray message_;
};

}  // namespace brainco_hand_touch_broadcaster

#endif  // BRAINCO_HAND_TOUCH_BROADCASTER__TOUCH_STATE_BROADCASTER_HPP_
