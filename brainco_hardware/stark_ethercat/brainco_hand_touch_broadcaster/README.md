# brainco_hand_touch_broadcaster

BrainCo 灵巧手触觉状态广播器插件，用于将硬件触觉传感器数据发布为 ROS2 消息。

- **版本**: 1.0.0
- **维护者**: Songjie Xiao &lt;songjie.xiao@joysonrobot.com&gt;

---

## 概述

本包以 `ros2_control` controller plugin 的形式实现了一个触觉状态广播器（`TouchStateBroadcaster`）。它从硬件接口的 state interface 中读取 5 根手指的触觉传感器数据，并以固定频率发布为 `brainco_hand_touch_msgs/msg/TouchStateArray` 消息。

支持左手（`left`）和右手（`right`）两种配置，同一机器人上可同时运行两个实例。

---

## 依赖项

| 包 | 用途 |
|---|---|
| `brainco_hand_touch_msgs` | 定义触觉状态消息类型 |
| `controller_interface` | ros2_control 控制器基类 |
| `hardware_interface` | 硬件接口抽象层 |
| `pluginlib` | 插件注册与加载 |
| `rclcpp` | ROS2 C++ 客户端库 |
| `rclcpp_lifecycle` | 生命周期节点支持 |

---

## 插件注册

```xml
<!-- touch_state_broadcaster_plugins.xml -->
<library path="brainco_hand_touch_broadcaster">
  <class
    name="brainco_hand_touch_broadcaster/TouchStateBroadcaster"
    type="brainco_hand_touch_broadcaster::TouchStateBroadcaster"
    base_class_type="controller_interface::ControllerInterface">
  </class>
</library>
```

在 `ros2_control` 配置文件中引用插件时，使用以下 `plugin` 字段：

```
brainco_hand_touch_broadcaster/TouchStateBroadcaster
```

---

## 参数说明

| 参数名 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| `hand_name` | `string` | 是 | `""` | 手的名称，必须为 `"left"` 或 `"right"` |
| `sensor_names` | `string[]` | 是 | `[]` | 5 个触觉传感器名称，顺序固定 |
| `topic_name` | `string` | 否 | `/<hand_name>_touch_states` | 发布话题名，为空则使用默认值 |

### sensor_names 规则

`sensor_names` 必须包含恰好 5 个元素，按以下顺序排列，命名格式为 `<hand_name>_<finger>_touch`：

```
left_thumb_touch
left_index_touch
left_middle_touch
left_ring_touch
left_pinky_touch
```

右手同理，将 `left` 替换为 `right`。顺序不可颠倒。

---

## State Interface

广播器订阅 **25 个** state interface（5 根手指 × 5 个字段），命名格式为：

```
<sensor_name>/<field>
```

例如左手：

```
left_thumb_touch/normal_force
left_thumb_touch/tangential_force
left_thumb_touch/direction
left_thumb_touch/proximity
left_thumb_touch/status
left_index_touch/normal_force
...
left_pinky_touch/status
```

广播器不占用任何 command interface。

---

## 发布话题

| 话题名 | 消息类型 | 说明 |
|---|---|---|
| `/<hand_name>_touch_states` | `brainco_hand_touch_msgs/msg/TouchStateArray` | 五指触觉状态数组 |

默认话题名：
- 左手：`/left_touch_states`
- 右手：`/right_touch_states`

---

## 消息结构

### `brainco_hand_touch_msgs/msg/TouchStateArray`

```
std_msgs/Header header       # 时间戳（由 update() 中的控制器时间填充）
string hand_name             # "left" 或 "right"
TouchFingerState[] fingers   # 5 根手指的状态列表
```

### `brainco_hand_touch_msgs/msg/TouchFingerState`

```
string  finger_name        # 手指名称：thumb / index / middle / ring / pinky
float32 normal_force       # 法向力（N）
float32 tangential_force   # 切向力（N）
float32 direction          # 力方向（rad）
uint32  proximity          # 接近度（原始值，来自 double 截断）
uint16  status             # 传感器状态标志位
```

**类型转换说明**：
- `normal_force`、`tangential_force`、`direction`：从 `double` 直接转换为 `float32`
- `proximity`：`double` → `uint32`，负数或非有限值映射为 `0`，超出上限截断为 `UINT32_MAX`
- `status`：`double` → `uint16`，负数或非有限值映射为 `0`，超出上限截断为 `UINT16_MAX`

---

## 配置示例

### ros2_control YAML 配置（左手）

```yaml
controller_manager:
  ros__parameters:
    left_touch_broadcaster:
      type: brainco_hand_touch_broadcaster/TouchStateBroadcaster

left_touch_broadcaster:
  ros__parameters:
    hand_name: left
    sensor_names:
      - left_thumb_touch
      - left_index_touch
      - left_middle_touch
      - left_ring_touch
      - left_pinky_touch
    # topic_name: /left_touch_states  # 可选，默认自动生成
```

### 同时运行左右手

```yaml
controller_manager:
  ros__parameters:
    left_touch_broadcaster:
      type: brainco_hand_touch_broadcaster/TouchStateBroadcaster
    right_touch_broadcaster:
      type: brainco_hand_touch_broadcaster/TouchStateBroadcaster

left_touch_broadcaster:
  ros__parameters:
    hand_name: left
    sensor_names:
      - left_thumb_touch
      - left_index_touch
      - left_middle_touch
      - left_ring_touch
      - left_pinky_touch

right_touch_broadcaster:
  ros__parameters:
    hand_name: right
    sensor_names:
      - right_thumb_touch
      - right_index_touch
      - right_middle_touch
      - right_ring_touch
      - right_pinky_touch
```

### 加载并激活控制器

```bash
ros2 control load_controller left_touch_broadcaster
ros2 control set_controller_state left_touch_broadcaster active
```

### 查看发布的话题

```bash
ros2 topic echo /left_touch_states
ros2 topic hz /left_touch_states
```

---

## 生命周期状态机

```
[unconfigured]
      │  on_init()       声明参数 hand_name / topic_name / sensor_names
      ↓
[inactive]  ←─── on_cleanup() / on_error()
      │  on_configure()  验证参数，创建 publisher，展开 state interface 名列表
      ↓
[active]
      │  on_activate()   检查 state interface 数量与名称，激活 publisher
      ↓  update()        读取触觉数据，填充消息，发布
      │  on_deactivate() 停用 publisher
      ↓
[inactive]
```

各阶段行为：

| 回调 | 行为 |
|---|---|
| `on_init` | 声明三个 ROS 参数 |
| `on_configure` | 读取并验证参数；为空时生成默认 topic_name；展开 25 个 state interface 名；初始化消息结构；创建 publisher |
| `on_activate` | 验证 publisher 存在；验证已分配的 state interface 数量与名称匹配；激活 publisher |
| `on_deactivate` | 停用 publisher（不清空状态） |
| `on_cleanup` | 重置所有内部状态（publisher、消息、名称列表） |
| `on_error` | 与 on_cleanup 行为相同 |
| `update` | 读取 25 个 state interface 值 → 填充消息 → 发布 |

---

## 错误处理

| 错误场景 | 处理方式 |
|---|---|
| `hand_name` 不是 `"left"` 或 `"right"` | `on_configure` 返回 `ERROR`，打印 RCLCPP_ERROR |
| `sensor_names` 数量不为 5 | `on_configure` 返回 `ERROR`，打印错误信息 |
| `sensor_names` 顺序或命名不符合规范 | `on_configure` 返回 `ERROR`，指出缺失的名称 |
| `on_activate` 时 publisher 未创建 | 返回 `ERROR` |
| `on_activate` 时 state interface 数量/名称不匹配 | 返回 `ERROR`，打印实际与期望数量 |
| `update` 时 state interface 不满足要求 | 返回 `return_type::ERROR`，打印错误 |
| `update` 时 publisher 未激活 | 直接返回 `OK`，不发布 |

---

## 构建

```bash
cd <workspace>
colcon build --packages-select brainco_hand_touch_broadcaster
```

### 运行单元测试

```bash
colcon test --packages-select brainco_hand_touch_broadcaster
colcon test-result --verbose
```

测试覆盖以下场景：
- `hand_name` 和 `sensor_names` 的参数验证
- state interface 名称展开顺序
- 消息初始化与数据填充
- `uint32`/`uint16` 安全转换（边界值、负数、溢出）
- `state_interface_configuration` 缓存机制
- `on_cleanup` / `on_error` 状态清理

---

## 代码结构

```
brainco_hand_touch_broadcaster/
├── include/
│   └── brainco_hand_touch_broadcaster/
│       └── touch_state_broadcaster.hpp   # 类定义与 detail 工具函数
├── src/
│   └── touch_state_broadcaster.cpp       # 生命周期回调与 update 实现
├── test/
│   └── test_touch_state_broadcaster.cpp  # GTest 单元测试
├── touch_state_broadcaster_plugins.xml   # pluginlib 插件描述
├── CMakeLists.txt
└── package.xml
```

### 关键常量（`detail` 命名空间）

```cpp
constexpr std::size_t kFingerCount = 5;
constexpr std::size_t kFieldCount  = 5;

// 手指顺序（固定）
kFingerNames = {"thumb", "index", "middle", "ring", "pinky"};

// 每个手指的字段顺序（固定）
kTouchFields = {"normal_force", "tangential_force", "direction", "proximity", "status"};
```
