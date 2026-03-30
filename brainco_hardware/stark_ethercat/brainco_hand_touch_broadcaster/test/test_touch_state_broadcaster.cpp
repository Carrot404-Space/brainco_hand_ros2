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

#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

#include "brainco_hand_touch_broadcaster/touch_state_broadcaster.hpp"

namespace brainco_hand_touch_broadcaster
{

struct TouchStateBroadcasterTestAccess
{
  static void set_runtime_state(
    TouchStateBroadcaster & broadcaster,
    const std::string & hand_name,
    const std::string & topic_name,
    const std::vector<std::string> & sensor_names,
    const std::vector<std::string> & state_interface_names)
  {
    broadcaster.hand_name_ = hand_name;
    broadcaster.topic_name_ = topic_name;
    broadcaster.sensor_names_ = sensor_names;
    broadcaster.state_interface_names_ = state_interface_names;
    detail::initialize_message(broadcaster.message_, hand_name, sensor_names);
  }

  static const std::vector<std::string> & state_interface_names(
    const TouchStateBroadcaster & broadcaster)
  {
    return broadcaster.state_interface_names_;
  }

  static const std::vector<std::string> & sensor_names(
    const TouchStateBroadcaster & broadcaster)
  {
    return broadcaster.sensor_names_;
  }

  static const std::string & hand_name(const TouchStateBroadcaster & broadcaster)
  {
    return broadcaster.hand_name_;
  }

  static const std::string & topic_name(const TouchStateBroadcaster & broadcaster)
  {
    return broadcaster.topic_name_;
  }

  static const brainco_hand_touch_msgs::msg::TouchStateArray & message(
    const TouchStateBroadcaster & broadcaster)
  {
    return broadcaster.message_;
  }
};

}  // namespace brainco_hand_touch_broadcaster

namespace
{

using brainco_hand_touch_broadcaster::detail::HandTouchValues;

TEST(TouchStateBroadcasterDetailTest, ValidatesHandNameAndSensorNames)
{
  std::string error_message;

  EXPECT_TRUE(brainco_hand_touch_broadcaster::detail::is_valid_hand_name("left"));
  EXPECT_TRUE(brainco_hand_touch_broadcaster::detail::is_valid_hand_name("right"));
  EXPECT_FALSE(brainco_hand_touch_broadcaster::detail::is_valid_hand_name("both"));

  EXPECT_EQ(
    brainco_hand_touch_broadcaster::detail::default_topic_name("left"),
    "/left_touch_states");
  EXPECT_EQ(
    brainco_hand_touch_broadcaster::detail::default_topic_name("right"),
    "/right_touch_states");

  EXPECT_FALSE(
    brainco_hand_touch_broadcaster::detail::validate_sensor_names(
      "left",
      {"left_thumb_touch", "left_index_touch", "left_middle_touch", "left_ring_touch"},
      &error_message));
  EXPECT_NE(error_message.find("exactly 5"), std::string::npos);

  error_message.clear();
  EXPECT_FALSE(
    brainco_hand_touch_broadcaster::detail::validate_sensor_names(
      "left",
      {"left_thumb_touch", "left_index_touch", "left_middle_touch", "left_ring_touch",
        "right_pinky_touch"},
      &error_message));
  EXPECT_NE(error_message.find("left_pinky_touch"), std::string::npos);

  error_message.clear();
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::detail::validate_sensor_names(
      "left", brainco_hand_touch_broadcaster::detail::expected_sensor_names("left"),
      &error_message));
  EXPECT_TRUE(error_message.empty());
}

TEST(TouchStateBroadcasterDetailTest, ExpandsTouchStateInterfacesInExpectedOrder)
{
  const auto interface_names = brainco_hand_touch_broadcaster::detail::expand_state_interface_names(
    brainco_hand_touch_broadcaster::detail::expected_sensor_names("left"));

  ASSERT_EQ(interface_names.size(), 25u);
  EXPECT_EQ(interface_names.front(), "left_thumb_touch/normal_force");
  EXPECT_EQ(interface_names[1], "left_thumb_touch/tangential_force");
  EXPECT_EQ(interface_names[2], "left_thumb_touch/direction");
  EXPECT_EQ(interface_names[3], "left_thumb_touch/proximity");
  EXPECT_EQ(interface_names[4], "left_thumb_touch/status");
  EXPECT_EQ(interface_names[20], "left_pinky_touch/normal_force");
  EXPECT_EQ(interface_names.back(), "left_pinky_touch/status");
}

TEST(TouchStateBroadcasterDetailTest, BuildsStateInterfaceConfigurationFromValidSensorNames)
{
  const auto sensor_names = brainco_hand_touch_broadcaster::detail::expected_sensor_names("right");
  const auto interface_names =
    brainco_hand_touch_broadcaster::detail::configured_state_interface_names(
    "right", sensor_names);

  ASSERT_EQ(interface_names.size(), 25u);
  EXPECT_EQ(interface_names.front(), "right_thumb_touch/normal_force");
  EXPECT_EQ(interface_names.back(), "right_pinky_touch/status");
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::detail::configured_state_interface_names("right", {})
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::detail::configured_state_interface_names(
      "invalid", sensor_names)
    .empty());
}

TEST(TouchStateBroadcasterDetailTest, MatchesExpectedInterfaceNamesExactly)
{
  const auto expected =
    brainco_hand_touch_broadcaster::detail::configured_state_interface_names(
    "left", brainco_hand_touch_broadcaster::detail::expected_sensor_names("left"));

  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::detail::has_expected_interface_names(expected, expected));

  auto actual = expected;
  actual.back() = "left_pinky_touch/proximity";
  EXPECT_FALSE(
    brainco_hand_touch_broadcaster::detail::has_expected_interface_names(expected, actual));
}

TEST(TouchStateBroadcasterDetailTest, InitializesAndPopulatesTouchStateMessage)
{
  brainco_hand_touch_msgs::msg::TouchStateArray message;
  const auto sensor_names = brainco_hand_touch_broadcaster::detail::expected_sensor_names("left");

  brainco_hand_touch_broadcaster::detail::initialize_message(message, "left", sensor_names);

  HandTouchValues touch_values{};
  touch_values[0] = {1.5, 2.5, 3.5, 4.0, 5.0};
  touch_values[1] = {6.0, 7.0, 8.0, 9.0, 10.0};
  touch_values[2] = {11.0, 12.0, 13.0, 14.0, 15.0};
  touch_values[3] = {16.0, 17.0, 18.0, 19.0, 20.0};
  touch_values[4] = {21.0, 22.0, 23.0, 24.0, 7.0};

  brainco_hand_touch_broadcaster::detail::assign_touch_values(message, touch_values);

  EXPECT_EQ(message.hand_name, "left");
  ASSERT_EQ(message.fingers.size(), 5u);
  EXPECT_EQ(message.fingers[0].finger_name, "thumb");
  EXPECT_EQ(message.fingers[1].finger_name, "index");
  EXPECT_EQ(message.fingers[2].finger_name, "middle");
  EXPECT_EQ(message.fingers[3].finger_name, "ring");
  EXPECT_EQ(message.fingers[4].finger_name, "pinky");
  EXPECT_FLOAT_EQ(message.fingers[0].normal_force, 1.5f);
  EXPECT_FLOAT_EQ(message.fingers[0].tangential_force, 2.5f);
  EXPECT_FLOAT_EQ(message.fingers[0].direction, 3.5f);
  EXPECT_EQ(message.fingers[0].proximity, 4u);
  EXPECT_EQ(message.fingers[0].status, 5u);
  EXPECT_EQ(message.fingers[4].status, 7u);
}

TEST(TouchStateBroadcasterDetailTest, ClampsUnsignedConversions)
{
  EXPECT_EQ(brainco_hand_touch_broadcaster::detail::safe_double_to_uint32(-1.0), 0u);
  EXPECT_EQ(brainco_hand_touch_broadcaster::detail::safe_double_to_uint32(42.0), 42u);
  EXPECT_EQ(
    brainco_hand_touch_broadcaster::detail::safe_double_to_uint16(70000.0),
    std::numeric_limits<std::uint16_t>::max());
}

TEST(TouchStateBroadcasterTest, ReturnsCachedStateInterfaceConfiguration)
{
  brainco_hand_touch_broadcaster::TouchStateBroadcaster broadcaster;
  const auto sensor_names = brainco_hand_touch_broadcaster::detail::expected_sensor_names("left");
  const auto state_interface_names =
    brainco_hand_touch_broadcaster::detail::configured_state_interface_names("left", sensor_names);

  brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::set_runtime_state(
    broadcaster, "left", "/left_touch_states", sensor_names, state_interface_names);

  const auto configuration = broadcaster.state_interface_configuration();

  EXPECT_EQ(
    configuration.type,
    controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(configuration.names, state_interface_names);
}

TEST(TouchStateBroadcasterTest, CleanupAndErrorClearCachedRuntimeState)
{
  brainco_hand_touch_broadcaster::TouchStateBroadcaster broadcaster;
  const auto sensor_names = brainco_hand_touch_broadcaster::detail::expected_sensor_names("right");
  const auto state_interface_names =
    brainco_hand_touch_broadcaster::detail::configured_state_interface_names("right", sensor_names);
  const rclcpp_lifecycle::State lifecycle_state;

  brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::set_runtime_state(
    broadcaster, "right", "/right_touch_states", sensor_names, state_interface_names);

  ASSERT_EQ(
    broadcaster.on_cleanup(lifecycle_state),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::state_interface_names(
      broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::sensor_names(broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::hand_name(broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::topic_name(broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::message(broadcaster)
    .fingers.empty());

  brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::set_runtime_state(
    broadcaster, "right", "/right_touch_states", sensor_names, state_interface_names);

  ASSERT_EQ(
    broadcaster.on_error(lifecycle_state),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::state_interface_names(
      broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::sensor_names(broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::hand_name(broadcaster)
    .empty());
  EXPECT_TRUE(
    brainco_hand_touch_broadcaster::TouchStateBroadcasterTestAccess::topic_name(broadcaster)
    .empty());
}

}  // namespace
