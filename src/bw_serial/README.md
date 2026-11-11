# bw_serial - Engineering Specification & README

## 1. 概述 (Overview)
`bw_serial` 是 BlueWorm 遥操作系统的 **底层通信与硬件接口层**。它负责通过串口（Serial Port）与机器人的嵌入式主控及力反馈控制器进行双向数据交换。该模块实现了私有二进制协议的封装与解析，是连接 ROS 2 高层逻辑与底层硬件执行器的桥梁。

---

## 2. 框架与职责边界 (Framework & Boundaries)

### 2.1 核心定位
位于系统的 **底层驱动层 (Hardware Interface Layer)**。它作为 ROS 2 节点运行，将高层的抽象控制指令（如位姿、速度）编码为二进制帧发送，并将硬件反馈的传感器数据（如电机实际位置、力矩）解码发布给高层。

### 2.2 职责范围 (In-Scope)
- **串口通信管理**: 使用 `serial_driver` 实现跨平台的异步串口读写。
- **协议编解码 (Codecs)**: 实现 `MantisCommand_t`、`MantisFeedback_t` 等二进制协议帧的打包与校验 (CRC16)。
- **实时控制流**: 以高频 (最高 1000Hz) 维持与下位机的心跳与指令更新。
- **模式切换服务**: 提供 `set_ctrl_src` 服务，允许动态切换机器人的控制源（如 VR 遥操作或 RViz 测试）。
- **力反馈透传**: 专门处理力反馈控制器的指令下发与反馈接收。

### 2.3 非职责范围 (Out-of-Scope)
- **运动学/动力学**: 不负责计算关节角度或轨迹（由 `bw_core` 负责）。
- **感知处理**: 不负责视觉或点云数据处理。

---

## 3. 文件夹结构 (Project Structure)
```text
bw_serial/
├── include/                  # 头文件目录
│   ├── protocol.hpp          # 二进制通信协议定义 (结构体、帧头尾)
│   ├── comm_utils.hpp        # 串口通用工具函数 (CRC校验等)
│   └── logger.hpp            # 串口数据日志工具
├── src/                      # 源代码目录
│   ├── mantis_comm_node.cpp  # 机器人主控通信节点
│   └── force_fdb_comm_node.cpp # 力反馈控制器通信节点
├── srv/                      # 服务定义
│   └── SetCtrlSrc.srv        # 设置控制源服务
├── msg/                      # 消息定义
│   ├── CtrlCmd.msg           # 控制指令自定义消息
│   └── ForceFdb.msg          # 力反馈数据自定义消息
├── launch/                   # 启动文件目录
│   └── mantis_comm.launch.py # 通信节点启动脚本
├── CMakeLists.txt            # 构建脚本
└── package.xml               # 包描述文件
```

---

## 4. 技术栈 (Technical Stack)
- **编程语言**: C++ 17
- **中间件**: ROS 2 Humble
- **库依赖**: 
    - `rclcpp`: ROS 2 核心 C++ 库。
    - `serial_driver`: 异步串口驱动库。
    - `rosidl_default_generators`: 用于生成自定义消息和服务。

---

## 5. 节点架构与参数 (Node Architecture & Parameters)

### 5.1 核心节点: `mantis_comm_node`
负责机器人全身关节、底盘及腰部的控制与状态反馈。
- **参数**:
    - `port_name` (string, default: "/dev/ttyUSB0"): 串口设备路径。
    - `baud_rate` (int, default: 9600): 波特率。
    - `publish_frequency` (int, default: 1000): 指令下发频率 (Hz)。
    - `protocol_version` (string, default: `v2`): 通信协议版本，支持 `v1` / `v2` / `v3` / `v3_chassis`，可在 launch 启动前选择。
    - `robot_version` (string, default: `2.0`): 机器人机械型号，决定滑台默认位置与限位等机械 profile，和 `protocol_version` 独立。
    - `auto_detect_hardware_shape` (bool, default: `false`): Standard 真机 `protocol_version:=v3` 是否分阶段探测整机与独立底盘 payload，并在收到 `0x22/37` 反馈后切换到 chassis 模式。
    - `left_gripper_invert` / `right_gripper_invert` (bool, default: `false`): 是否在发送给下位机前反相夹爪开合命令。
    - `pelvis_default_height_mm` (double): `v2/v3` 内部滑台目标位置估计与启动初始化目标，单位 mm。Standard 整机下位机采用相对位置，默认 `-100.0`；自动识别到底盘形态后固定使用 `1200.0`。
    - `pelvis_min_height_mm` (double): `v2/v3` 内部滑台目标下限，单位 mm。Standard 整机为 `-500.0`，底盘形态为 `900.0`。
    - `pelvis_max_height_mm` (double, default: `1500.0`): `v2/v3` 内部滑台目标上限，单位 mm。Standard 整机为 `0.0`，底盘形态为 `1500.0`。
    - `pelvis_jog_max_speed_mm_s` (double, default: `200.0`): `v2/v3` 将 VR `pelvis_speed` 输入转换为位置 jog 时使用的最大速度，单位 mm/s。
    - `pelvis_input_deadzone` (double, default: `0.1`): `v2/v3` 对 `pelvis_speed` 输入应用的死区。
    - `pelvis_startup_init_enabled` (bool, default: `true`): `v2/v3` 上电后是否先把滑台移动到 `pelvis_default_height_mm`。
    - `pelvis_startup_init_tolerance_mm` (double, default: `10.0`): 启动滑台初始化的到位容差，反馈高度进入该范围后停止初始化速度。
    - `pelvis_startup_init_timeout_sec` (double, default: `8.0`): 启动滑台初始化最长持续时间，超时后停止初始化速度，避免长期占用滑台控制。
    - `waist_default_pos_deg` (double, default: `0.0`): `protocol_version:=v3` 时上半身前后弯腰的初始目标角度，单位 deg。
    - `waist_min_pos_deg` (double, default: `-90.0`): `protocol_version:=v3` 时上半身前后弯腰目标角度下限，单位 deg。
    - `waist_max_pos_deg` (double, default: `5.0`): `protocol_version:=v3` 时上半身前后弯腰目标角度上限，单位 deg。
    - `waist_jog_max_speed_deg_s` (double, default: `20.0`): `protocol_version:=v3` 时上半身前后弯腰最大角速度限制，单位 deg/s。
    - `waist_input_deadzone` (double, default: `0.1`): `protocol_version:=v3` 时 VR 弯腰 jog 输入死区。
    - `v3_fixed_waist_pos_deg` (double, default: `0.0`): `protocol_version:=v3` 时固定写入控制帧的腰部目标角度，单位 deg。
    - `v3_fixed_waist_max_velocity_deg_s` (double, default: `0.0`): `protocol_version:=v3` 时固定写入控制帧的腰部最大角速度，单位 deg/s。
    - `v3_feedback_publish_frequency_hz` (double, default: `250.0`): V3 反馈 ROS 话题发布上限。串口仍逐帧解码并更新内部状态；Standard bringup 显式配置为 `250 Hz`，用于降低组件容器回调压力。

### 5.2 辅助节点: `force_fdb_comm_node`
专门负责处理 VR 手柄的力反馈震动/阻力控制指令。

---

## 6. 接口定义 (Interface Definitions)

### 6.1 订阅话题 (Subscribed Topics)
- `/Teleop/joint_angle_solution/smooth` (`sensor_msgs/JointState`): 平滑后的手臂关节角度指令。
- `/Teleop/head_pose` (`sensor_msgs/JointState`): 头部 Pitch/Yaw 角度。
- `/Teleop/gripper_pos` (`sensor_msgs/JointState`): 夹爪开合位置。
- `/Teleop/cmd_vel` (`geometry_msgs/Twist`): 底盘运动速度。
- `/Teleop/pelvis_speed` (`std_msgs/Float32`): VR 滑台 jog 输入。`v1` 直接按历史速度控制使用；`v2/v3` 将其解释为位置 jog 意图。
- `/Teleop/pelvis_height` (`std_msgs/Float32`): SDK 滑台绝对高度目标，单位 m，按 URDF/SDK 零位的相对位移；`v2/v3` 使用。
- `/Teleop/pelvis_max_velocity` (`std_msgs/Float32`): SDK 滑台最大速度限制，单位 mm/s；只限制速度，不改变高度目标。
- `/Teleop/waist_speed` (`std_msgs/Float32`): VR 上半身前后弯腰 jog 输入。当前仅 `protocol_version:=v3` 消费；`v1/v2` 忽略。
- `/Teleop/waist_angle` (`std_msgs/Float32`): SDK 上半身前后弯腰绝对角度目标，单位 rad；当前仅 `protocol_version:=v3` 消费。
- `/Teleop/waist_max_velocity` (`std_msgs/Float32`): SDK 上半身前后弯腰最大角速度限制，单位 deg/s；当前仅 `protocol_version:=v3` 消费。

`mantis_comm_node` 当前同时兼容两套关节命名输入：
- 历史 `snake_case`：如 `left_shoulder_pitch_joint`
- formal URDF `CamelCase`：如 `L_Shoulder_Pitch_Joint`

夹爪命令在串口层支持独立反相。默认配置下：
- ROS / RViz 侧保持 `0.0=松开, 1.0=按下扳机最大`
- 下位机同样按 `0.0=打开, 1.0=闭合` 解释，不做反相
- 如果某台机器人硬件语义相反，可单独把 `left_gripper_invert/right_gripper_invert` 打开

### 6.2 发布话题 (Published Topics)
- `/joint_states_fdb` (`sensor_msgs/JointState`): 从硬件反馈的全身关节实际位置。
- `/force_feedback` (`sensor_msgs/JointState`): 硬件层反馈的力矩数据。
- `/wave_hand_switch` (`std_msgs/Bool`): 硬件触发的摆手模式开关。
- `/debug/serial_tx` (`std_msgs/Float64MultiArray`): 串口发送调试信息，包含 `[joint_stamp, tx_time]` 用于延迟分析。
- `/mantis_feedback_v2` (`bw_serial/MantisFeedbackV2`): V2 协议完整反馈数据。
- `/joint_states_fdb_V3` (`sensor_msgs/JointState`): V3 整机反馈；Standard 自动识别到底盘形态或 `protocol_version:=v3_chassis` 时仍使用此 topic，但只发布 `pelvis_joint`、`odom_x_joint`、`odom_y_joint`、`chassis_omega_joint`、`robot_heading_joint` 五个底盘/滑台字段。

### 6.4 V2 话题 (New V2 Topics)
- `/Teleop/mantis_command_v2` (`bw_serial/MantisCommandV2`): V2 协议控制数据。
- `/mantis_feedback_v2` (`bw_serial/MantisFeedbackV2`): V2 协议反馈数据。

> 现有 V1 话题与消息保持不变，`protocol_version:=v1` 仍按原有流程工作。

### 6.3 服务 (Services)
- `set_ctrl_src` (`bw_serial/srv/SetCtrlSrc`): 设置控制源标志位 (如 `0b10` 为 VR 遥操作)。

---

## 7. 行为语义与协议 (Behavioral Semantics)

### 7.1 二进制协议帧
| 字段 | 类型 | 描述 |
| :--- | :--- | :--- |
| `frame_header` | uint8_t | 指令帧: `0x1F`, 反馈帧: `0x2F` |
| `ctrl_src_flag` | uint8_t | 控制源标志位，决定底层控制逻辑切换 |
| `payload` | float[] | 包含底盘、手臂、头部、夹爪的所有控制/反馈数据 |
| `frame_tail` | uint8_t | 固定为 `0x1E` |
| `crc16` | uint16_t | CRC16 校验码，确保数据完整性 |

> `protocol_version:=v2` 时，控制帧按照 `src/嵌入式&工控机-控制帧.xlsx` 打包，帧长度固定 `256` 字节。
> 
> `protocol_version:=v2` 时，反馈帧按照 `src/嵌入式&工控机-反馈帧.xlsx` 解析，总长度 `256` 字节（含中间分包头尾标记）。
>
> `protocol_version:=v1` 时，`Teleop/pelvis_speed` 直接透传到 `pelvis_velocity`，保持历史速度控制行为不变。
>
> `protocol_version:=v2/v3` 时，节点支持两种滑台输入：
> - `Teleop/pelvis_height` 直接写入协议中的 `pelvis_height` 绝对高度目标。
> - `Teleop/pelvis_speed` 仅作为 VR jog 输入，在串口发送周期内维护一个内部滑台目标高度估计。
> - 目标初值由 `pelvis_default_height_mm` 给出；Standard 使用下位机相对位置，从 `-100mm` 启动
> - 节点上电后会先进入滑台启动初始化：持续发送 `pelvis_default_height_mm` 和 `pelvis_jog_max_speed_mm_s`，直到反馈高度进入 `pelvis_startup_init_tolerance_mm` 或达到 `pelvis_startup_init_timeout_sec`
> - 启动初始化只在 `v2/v3` 且下位机控制标志已上电后生效；EMERGENCY/掉电状态不会驱动滑台
> - 收到显式 `Teleop/pelvis_height` 或有效 `Teleop/pelvis_speed` 后会取消启动初始化，优先执行外部滑台控制输入
> - VR jog 从停止进入运动或切换上下方向时，会先把内部目标高度同步到最新下位机反馈高度，避免从旧目标高度起步导致短暂反向滑动
> - 每周期按 `target += jog_speed * dt` 积分
> - 目标会被限制在 `[pelvis_min_height_mm, pelvis_max_height_mm]`；Standard 的范围为 `[-500,0]mm`
> - 回中死区内停止积分，并把协议中的最大速度置 `0`
>
> `protocol_version:=v3` 时，节点支持两种上半身前后弯腰输入：
> - `Teleop/waist_angle` 直接写入协议中的 `waist_pos` 绝对角度目标。
> - `Teleop/waist_speed` 仅作为 VR jog 输入，内部维护腰部目标角度 `waist_pos`
> - 每周期按 `target += jog_speed * dt` 积分
> - 目标角度被限制在 `[waist_min_pos_deg, waist_max_pos_deg]`
> - `waist_max_vel` 仅作为本周期允许的最大角速度限制，松手后归零
>
> `protocol_version:=v3` 时，底盘速度 `vx/vy/omega` 来自 `Teleop/cmd_vel`，摇杆只控制最大速度命令；底盘加速度限制固定写入：
> - `max_acc_x = 1.5 m/s^2`
> - `max_acc_y = 1.5 m/s^2`
> - `max_acc_omega = 1.5 rad/s^2`
>
> `v3_fixed_waist_*` 现仅作为兼容旧配置的默认初值来源；真正发送给下位机的腰部字段由 jog 控制器动态生成。
>
> `protocol_version:=v3` 的线协议格式为 `55 AA | TYPE | LEN_L | LEN_H | PAYLOAD | CRC_L | CRC_H`。
> 其中 `TYPE=0x01` 表示控制帧，`TYPE=0x02` 表示反馈帧；`LEN` 仅表示 payload 长度，使用小端序；
> `CRC` 使用 `CRC-16/MODBUS`，计算范围为 `TYPE + LEN_L + LEN_H + PAYLOAD`，不包含帧头 `55 AA`。

> `protocol_version:=v3_chassis` 使用相同的 V3 外层帧，但控制/反馈类型分别为 `0x21/0x22`，payload 固定 37 字节。控制 payload 包含软件开关、`vx/vy/omega`、底盘三轴最大加速度、滑台高度、滑台最大速度和滑台加速度；反馈 payload 包含状态标志、底盘速度、里程计、航向角以及滑台高度/速度。该模式不消费手臂关节、头部或夹爪输入，滑台高度范围由 bringup 固定为 `900~1500 mm`。

> `robot_version:=standard` 且 `protocol_version:=v3`、`auto_detect_hardware_shape:=true` 时，节点会自动探测硬件形态：先连续发送一段合法、静止的 `0x21`（独立底盘）控制帧；在该阶段没有收到 `0x22/37` 反馈时，再连续发送 `0x01`（整机）控制帧，并在两种探测阶段之间轮换。探测帧只带软件上电位 `0x01`，底盘探测阶段 `vx/vy/omega` 固定为零。节点接受 `0x02/248` 或 `0x22/37` 反馈，收到反馈后切换到对应 payload；若两阶段合计约 1000 帧仍无反馈，则按独立底盘处理，发布 `system/available_arms=0`，切换到 `0x21/0x22` 和滑台范围 `900~1500 mm`，保证不依赖反馈也能进入 VR 控制。直接使用 `protocol_version:=v3_chassis` 时不执行探测，始终使用 `0x21/0x22`。

> `auto_detect_hardware_shape` 只负责串口形态与 `system/available_arms`。bringup 在收到判型结果前仍按 Standard 整机建立控制链，不能因为存在底盘候选形态而提前关闭腕部输入、自动复位、控制激活门控、Standard smoother 或硬件反馈门控。识别为底盘后，消费者依据 `available_arms=0` 停止手臂输出。

> 自动切换到底盘形态后，`joint_states_fdb_V3` 只包含 `pelvis_joint`、`odom_x_joint`、`odom_y_joint`、`chassis_omega_joint` 和 `robot_heading_joint` 五个字段；滑台目标范围为 `900~1500 mm`，默认目标为 `1200 mm`。

### 7.2 异步接收逻辑
节点使用 `async_receive_message` 在独立线程中处理串口输入，通过查找帧头尾并校验 CRC 的方式确保解析出的 `MantisFeedback_t` 数据包准确无误。

---

## 8. 编译与运行 (Build & Run)

```bash
# 编译
colcon build --packages-select bw_serial

# 启动通信节点
ros2 launch bw_serial mantis_comm.launch.py

# 启动时选择协议版本
ros2 launch bw_serial mantis_comm.launch.py protocol_version:=v1

# 独立底盘 + 滑台
ros2 launch bw_serial mantis_comm.launch.py \
  protocol_version:=v3_chassis port_name:=/dev/ttyACM0 baud_rate:=115200
```

## V3 下位机调试发布器

如果你要单独验证 `v3` 串口链路，而不启动完整 VR 遥操作，可以直接启动：

```bash
ros2 launch bw_serial v3_test_publisher.launch.py port_name:=/dev/ttyACM0 baud_rate:=115200 publish_frequency:=10
```

默认行为：
- `v3_test_publisher.py` 周期发布 `Teleop/joint_angle_solution/smooth`
- 周期发布 `Teleop/head_pose`、`Teleop/gripper_pos`、`Teleop/cmd_vel`
- 自动调用 `set_ctrl_src=1`，让 `mantis_comm_node` 走 V3 软件开机控制位

常用参数：

```bash
ros2 launch bw_serial v3_test_publisher.launch.py \
  port_name:=/dev/ttyACM0 \
  publish_frequency:=10 \
  auto_enable:=false \
  active_joint:=3 \
  joint_amplitude:=0.2 \
  joint_period_sec:=4.0
```

其中 `active_joint` 使用 `0~13`，对应测试单个手臂关节的小幅正弦摆动；`-1` 表示全部保持零位。`auto_enable:=false` 时测试节点不会主动调用 `set_ctrl_src` 上电。

---

## 9. 工程约束 (Compliance & Enforcement)
- **内存安全**: 严禁在 `transmit_timer_callback` 中进行动态内存分配。
- **数据对齐**: 使用 `#pragma pack(1)` 确保协议结构体在不同平台间的二进制兼容性。
- **实时性**: 串口接收缓冲区应及时清理，避免数据积压导致控制延迟。
