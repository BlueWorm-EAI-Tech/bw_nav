# bw_interface - Engineering Specification & README

## 1. 概述 (Overview)
`bw_interface` 是 BlueWorm 遥操作系统的 **通信接口定义包**。它不包含任何可执行节点，而是作为整个系统的“语言标准”，定义了所有模块通用的消息 (Message) 和服务 (Service) 协议。它是系统解耦的关键，确保了不同语言（C++/Python）编写的模块能够基于统一的数据结构进行交互。

---

## 2. 框架与职责边界 (Framework & Boundaries)

### 2.1 核心定位
位于系统的 **基础架构层 (Foundation Layer)**。所有涉及自定义通信的包（如 `bw_core`, `bw_action_module`）都必须依赖此包。

### 2.2 职责范围 (In-Scope)
- **消息定义 (msg)**: 定义机器人状态、遥操作模式、VR 设备输入等标准数据结构。
- **服务定义 (srv)**: 定义数据录制控制、状态查询等交互协议。
- **代码生成**: 自动生成 C++ 和 Python 的头文件/库文件。

### 2.3 非职责范围 (Out-of-Scope)
- **逻辑实现**: 不包含任何节点、回调函数或业务逻辑。
- **第三方接口转换**: 不负责将外部 SDK（如 Meta Quest）直接转换为此接口（由感知层包负责）。

---

## 3. 文件夹结构 (Project Structure)
```text
bw_interface/
├── msg/                       # 消息定义目录 (.msg)
│   ├── VR.msg                 # VR 设备完整输入状态
│   ├── HandJoints.msg         # PICO 手部追踪 26 关节完整位姿
│   ├── SystemStatus.msg       # 系统生命周期状态与控制源信息
│   ├── RobotIdentity.msg      # 机器人身份广播信息（IP + SN）
│   ├── ForceHandle.msg        # ESP32 力反馈手柄原始输入帧
│   ├── ForceHandleTauCmd.msg  # ESP32 力反馈手柄 MIT 力矩命令下行给 VR 的合并消息
│   ├── MotionProfile.msg      # 动作文件按段 Ruckig 运动约束
│   ├── ArmCommand.msg         # Router 后统一手臂命令
│   ├── ArmCommandStatus.msg   # SDK 手臂命令生命周期状态
│   └── RobotCommand.msg       # Router 后统一整机命令
├── srv/                       # 服务定义目录 (.srv)
│   ├── StartRecord.srv        # 开始数据录制
│   ├── StopRecord.srv         # 停止数据录制
│   ├── PauseRecord.srv        # 暂停数据录制
│   ├── ResumeRecord.srv       # 恢复数据录制
│   └── GetStatus.srv          # 获取当前系统状态
├── package.xml                # 包元数据及接口依赖
└── CMakeLists.txt             # 接口生成配置
```

---

## 4. 技术栈 (Technical Stack)
- **中间件**: ROS 2 Humble
- **构建系统**: `ament_cmake`
- **接口生成器**: `rosidl_default_generators`
- **依赖基础接口**: `std_msgs`, `geometry_msgs`, `sensor_msgs`, `builtin_interfaces`

---

## 5. 接口定义详情 (Interface Definitions)

### 5.1 核心消息 (Messages)

#### `VR.msg`
定义了 VR 遥操作所需的完整感知数据：
- **位姿**: `head_pose`, `left_pose`, `right_pose` (含位置与姿态)。
- **交互**: 左右手的 `grip`, `trigger`, `joystick`, `button` 状态。
- **状态**: 设备连接情况 `left_connected`, `right_connected`。
- **模拟量**: `left_grip_value`, `right_grip_value`, `left_trigger_value`, `right_trigger_value`，用于承载 Unity XR `CommonUsages.grip/trigger` 的 0~1 线性输入，同时保留原有 bool 字段兼容现有下游按钮逻辑。

#### `HandJoints.msg`
定义了 PICO 手部追踪的原始关节完整位姿数据：
- **追踪状态**: `left_hand_tracked`, `right_hand_tracked` 表示左右手当前是否被 PICO SDK 成功追踪。
- **关节名称**: `joint_names` 固定包含 26 个 PICO `HandJoint` 枚举名称，用于解释左右手关节数组的索引含义。
- **关节位姿**: `left_hand_joints`, `right_hand_joints` 固定包含 26 个 `geometry_msgs/Pose`，顺序与 PICO `HandJoint` 枚举一致；每个关节包含 `position` 和 `orientation`。
- **坐标系**: 位姿由 Unity 端转换为 ROS FLU 坐标系，`header.frame_id` 与 VR bridge 参数 `frame_id` 保持一致，默认 `world`。
- **传输语义**: 手部关节通过独立 UDP 包传输，不追加到 `VR.msg` 对应的控制 UDP 包，避免影响现有控制链路兼容性。

#### `SystemStatus.msg`
系统状态、控制源与运动状态定义：
- **状态枚举**:
  - `SYS_UNINITIALIZED = 0` (系统未就绪)
  - `SYS_STANDBY = 1` (就绪/待机)
  - `SYS_ACTIVE = 2` (运行中)
  - `SYS_EMERGENCY = 3` (紧急停止)
  - `SYS_RESET = 4` (复位中)
  - `SYS_MENU = 5` (VR 菜单配置状态，VR 控制输入暂停但菜单命令仍可生效)
- **控制源枚举**:
  - `SRC_NONE = 0`, `SRC_VR = 1`, `SRC_SDK = 2`, `SRC_MOTION = 3`
- **运动状态 (模拟字典)**:
  - `motion_names`: 包含关节名称 (如 `left_shoulder_pitch`)、`waist`、`chassis`、`head`。
  - `motion_states`: 对应名称的运动状态 (0: 静止, 1: 运动)。
- **故障诊断**:
  - `missing_devices`: 未连接的硬件设备列表。
  - `missing_nodes`: 未启动的核心节点列表。

#### `RobotIdentity.msg`
机器人发现广播信息：
- `ip`: 机器人当前局域网 IP 地址。
- `sn`: 机器人 SN 码（如 `BW_4TEOGD`）。

#### `ForceHandle.msg`
ESP32 力反馈手柄 UDP bridge 发布的原始输入数据：
- `source_ip` / `source_port`: UDP 数据来源地址，便于多设备联调时定位发送端。
- `position`: 电机任务侧位置数据，单位由 ESP32 固件定义，input 节点通过参数归一化到 `0~1`。
- `joy_x` / `joy_y`: 摇杆有符号原始值（`int16`），ESP32 固件通常为 **±1000** 满量程。
- `joy_button`: 摇杆按键状态，`0=松开`，`1=按下`。
- `key_bitmap`: 四个独立按键位图，bit0~bit3 对应 `KEY_1`~`KEY_4`。
- `key1`~`key4`: `key_bitmap` 解包后的布尔按键状态。
- `crc`: 原始帧 CRC8 字节，仅用于调试和链路记录。

#### `ForceHandleTauCmd.msg`
ESP32 力反馈手柄上报的 MIT 力矩命令合并消息，供 Unity/PICO 通过 ROS-TCP 独立订阅：
- `header`: 发布时刻，`frame_id` 固定为 `force_handle`。
- `left_tau_cmd` / `right_tau_cmd`: 左右手最近一次 `motorDataFromTask.tau_cmd`，原样保留 ESP32 `float32` 数值。
- `left_valid` / `right_valid`: 对应侧是否在 `control_timeout_sec` 内仍有有效 UDP 输入，VR 端可据此忽略断流侧旧值。

#### `MotionProfile.msg`
动作文件回放按段下发给 `smoother_node` 的 Ruckig 约束：
- `header`: 发布时刻。
- `mode`: Web Action Studio 生成策略，当前为 `smooth` 或 `aggressive`。
- `max_velocity`: 该段最大关节速度，单位 `rad/s`。
- `max_acceleration`: 该段最大关节加速度，单位 `rad/s^2`。
- `max_jerk`: 该段最大关节加加速度，单位 `rad/s^3`。
- `duration_sec`: 该段按当前播放倍率折算后的执行时长，单位秒。

#### `ArmCommand.msg`
Router 后统一手臂命令，用于把不同输入源收敛成一条语义清晰的手臂控制接口：
- `COMMAND_JOINT`: 关节角命令，`joints` 可携带 1~14 个手臂关节。
- `COMMAND_POSE_ABS`: 机器人坐标系下的末端绝对位姿，使用 `left_pose/right_pose` 和对应 valid 标志。
- `COMMAND_POSE_REL`: 末端相对增量，使用 `[dx, dy, dz, droll, dpitch, dyaw]`；平移为全局增量，旋转为局部增量。
- `source`: 当前输入源名，仅用于日志、诊断和链路追踪，不参与控制源仲裁。
- `command_id`: SDK 阻塞等待链路的命令追踪 ID，新增在消息末尾；非 SDK 输入源可为空。
- `motion_profile_valid/motion_profile`: 可选运动约束。SDK 可逐条命令覆盖速度、加速度和 jerk；Router 也可按来源补默认约束，并通过 `RobotCommand.motion_profile` 传给 smoother。

#### `ArmCommandStatus.msg`
SDK 手臂命令生命周期状态，用于让 SDK `block=True` 等待同一条命令真正执行完成：
- 状态枚举：`STATUS_RECEIVED / STATUS_ROUTED / STATUS_RESOLVED / STATUS_FAILED / STATUS_COMPLETED / STATUS_TIMEOUT`。
- `command_id`: 对应 `ArmCommand.command_id`。
- `source`: 命令来源，首版主要使用 `sdk`。
- `message`: 状态说明或失败原因。
- `target`: resolver 解析出的最终目标关节，`name/position` 使用 formal URDF CamelCase，单位 rad。

#### `RobotCommand.msg`
Router 后统一整机控制命令，用于把 VR、SDK、文件动作和 Web Action Studio 收敛成一个正式控制 topic：
- `source`: 当前输入源名，仅用于日志、诊断和 SDK command_id 追踪；下游执行节点不得用它做控制分支。
- `source` 只表示真实控制源，例如 `vr/force_handle/sdk/file/action_studio`；公共 `bw_teleop` 产生的夹爪、底盘、滑台和腰部命令在 router 后会归属为当前激活的 `vr` 或 `force_handle`，不形成独立 `teleop` 执行源。
- `arm`: 内嵌 `ArmCommand`，`COMMAND_NONE` 表示本帧没有手臂更新。
- `head/gripper/cmd_vel`: 头部、夹爪和底盘命令，分别由 `*_valid` 标志控制是否生效。
- `pelvis_speed/waist_speed`: 滑台和前后弯腰 jog 输入。
- `pelvis_height/pelvis_max_velocity/waist_angle/waist_max_velocity`: 绝对目标和速度限制。
- `motion_profile`: 显式运动约束，主要由动作文件、Web Action Studio 或 SDK 手臂命令提供。下游 smoother 只消费该显式约束，不直接按 `source` 分支。
- `joint_target`: Action Studio/file 兼容仿真或动作预览用的完整关节目标；Router 只把它放入 `RobotCommand`，legacy `ctrl/joint_target` 由 dispatcher 发布。

### 5.2 核心服务 (Services)
- **数据录制族**: `StartRecord`, `StopRecord`, `PauseRecord`, `ResumeRecord`。
  - **Request**: 空。
  - **Response**: `bool success`, `string message`。
- **状态查询**: `GetStatus`。
  - **Response**: `string status`。

---

## 6. 编译与使用 (Build & Usage)

### 6.1 编译
```bash
colcon build --packages-select bw_interface
```

### 6.2 在其他包中引用
**package.xml**:
```xml
<depend>bw_interface</depend>
```

**C++**:
```cpp
#include "bw_interface/msg/vr.hpp"
#include "bw_interface/msg/motion_profile.hpp"
#include "bw_interface/srv/start_record.hpp"
```

**Python**:
```python
from bw_interface.msg import VR
from bw_interface.msg import MotionProfile
from bw_interface.srv import StartRecord
```

---

## 7. 工程约束 (Compliance & Enforcement)
- **前向兼容**: 严禁随意删除或修改已有的消息字段，新增字段应添加在末尾。
- **命名规范**: 遵循 ROS 2 官方命名规范（消息/服务名称使用大驼峰 `PascalCase`，字段使用小写加下划线 `snake_case`）。
- **依赖最小化**: 仅包含必要的标准接口依赖，保持包的轻量化。
