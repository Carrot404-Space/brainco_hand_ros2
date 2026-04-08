# BrainCo Hand EtherCAT Touch Integration

This directory contains the EtherCAT driver stack for BrainCo Revo2 hands, including the touch state broadcaster and touch message definitions introduced for ros2_control based touch publishing.

## Packages

| Package | Description |
|---|---|
| `brainco_hand_ethercat_driver` | Revo2 EtherCAT ros2_control hardware interface, launch files, and controller configuration |
| `brainco_hand_touch_broadcaster` | `ros2_control` controller plugin that publishes touch sensor state |
| `brainco_hand_touch_msgs` | ROS 2 message definitions for hand touch state output |
| `stark_ethercat_interface` | EtherCAT communication interface layer |
| `stark_ethercat_driver` | EtherCAT driver implementation |

## Touch Integration Overview

The touch data path is:

1. EtherCAT driver reads raw touch sensor data from the hand.
2. `brainco_hand_ethercat_driver` exports touch values as ros2_control state interfaces.
3. `brainco_hand_touch_broadcaster/TouchStateBroadcaster` reads those state interfaces.
4. The broadcaster publishes `brainco_hand_touch_msgs/msg/TouchStateArray` on a ROS 2 topic.

Each hand exports 25 touch state interfaces:

```text
<hand>_<finger>_touch/normal_force
<hand>_<finger>_touch/tangential_force
<hand>_<finger>_touch/direction
<hand>_<finger>_touch/proximity
<hand>_<finger>_touch/status
```

Example:

```text
right_thumb_touch/normal_force
right_thumb_touch/tangential_force
right_thumb_touch/direction
right_thumb_touch/proximity
right_thumb_touch/status
```

## Build

Build the packages in this directory:

```bash
cd ~/brainco_ws/src/brainco_hand_ros2/brainco_hardware/stark_ethercat
colcon build --symlink-install
```

Or build only the touch-related packages and their driver dependencies:

```bash
cd ~/brainco_ws
colcon build --packages-select \
  brainco_hand_touch_msgs \
  brainco_hand_touch_broadcaster \
  brainco_hand_ethercat_driver \
  stark_ethercat_interface \
  stark_ethercat_driver \
  --symlink-install
```

After building:

```bash
source ~/brainco_ws/install/setup.bash
```

## Launch

Start the right hand system:

```bash
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=right
```

Start the left hand system:

```bash
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=left
```

The launch file now spawns these controllers automatically:

| Hand | Joint state broadcaster | Touch broadcaster | Trajectory controller |
|---|---|---|---|
| Left | `joint_state_broadcaster` | `left_touch_state_broadcaster` | `left_revo2_hand_controller` |
| Right | `joint_state_broadcaster` | `right_touch_state_broadcaster` | `right_revo2_hand_controller` |

## Controller Configuration

Touch broadcaster configuration is defined in:

- `brainco_hand_ethercat_driver/config/revo2_left_controllers.yaml`
- `brainco_hand_ethercat_driver/config/revo2_right_controllers.yaml`

Right hand example:

```yaml
controller_manager:
  ros__parameters:
    right_touch_state_broadcaster:
      type: brainco_hand_touch_broadcaster/TouchStateBroadcaster

right_touch_state_broadcaster:
  ros__parameters:
    hand_name: right
    topic_name: /right_touch_states
    sensor_names:
      - right_thumb_touch
      - right_index_touch
      - right_middle_touch
      - right_ring_touch
      - right_pinky_touch
```

`sensor_names` must contain exactly five entries in finger order:

```text
thumb, index, middle, ring, pinky
```

## Published Topics

| Topic | Message Type | Description |
|---|---|---|
| `/left_touch_states` | `brainco_hand_touch_msgs/msg/TouchStateArray` | Left hand touch state |
| `/right_touch_states` | `brainco_hand_touch_msgs/msg/TouchStateArray` | Right hand touch state |

Check the published touch topic:

```bash
ros2 topic echo /right_touch_states
```

Check publish rate:

```bash
ros2 topic hz /right_touch_states
```

## Message Definitions

`brainco_hand_touch_msgs/msg/TouchStateArray`

```text
std_msgs/Header header
string hand_name
brainco_hand_touch_msgs/TouchFingerState[] fingers
```

`brainco_hand_touch_msgs/msg/TouchFingerState`

```text
string finger_name
float32 normal_force
float32 tangential_force
float32 direction
uint32 proximity
uint16 status
```

Inspect the installed interfaces:

```bash
ros2 interface show brainco_hand_touch_msgs/msg/TouchStateArray
ros2 interface show brainco_hand_touch_msgs/msg/TouchFingerState
```

## Verification

List loaded controllers:

```bash
ros2 control list_controllers
```

Expected touch controller names:

```text
left_touch_state_broadcaster
right_touch_state_broadcaster
```

If the touch topic is missing or empty, verify:

1. `brainco_hand_touch_msgs` and `brainco_hand_touch_broadcaster` were built and sourced.
2. The correct `revo2_<hand>_controllers.yaml` file was loaded.
3. The touch broadcaster is in `active` state.
4. The hardware interface is exporting the expected `*_touch/*` state interfaces.

## Related Package Documentation

- `brainco_hand_ethercat_driver/README.md`
- `brainco_hand_touch_broadcaster/README.md`
