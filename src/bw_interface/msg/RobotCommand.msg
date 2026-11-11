# Router 后统一整机控制命令。
# control_router_node 只发布本消息，后级 dispatcher/resolver 再适配到现有执行 topic。

std_msgs/Header header

# 当前输入源名，仅用于日志、诊断和 SDK command_id 链路追踪。
# 下游执行节点不得用它做控制分支。
string source

# 手臂命令。arm.command_type=COMMAND_NONE 表示本帧没有手臂更新。
bw_interface/ArmCommand arm

# 头部与夹爪关节目标。
bool head_valid
sensor_msgs/JointState head
bool gripper_valid
sensor_msgs/JointState gripper

# 底盘速度。
bool cmd_vel_valid
geometry_msgs/Twist cmd_vel

# 滑台/腰部 jog 输入。
bool pelvis_speed_valid
float32 pelvis_speed
bool waist_speed_valid
float32 waist_speed

# 滑台/腰部绝对目标与速度限制。
bool pelvis_height_valid
float32 pelvis_height
bool pelvis_max_velocity_valid
float32 pelvis_max_velocity
bool waist_angle_valid
float32 waist_angle
bool waist_max_velocity_valid
float32 waist_max_velocity

# 显式运动约束。主要由动作文件/Web Action Studio 这类“命令语义”上游提供。
bool motion_profile_valid
bw_interface/MotionProfile motion_profile

# 兼容仿真/动作预览 JointState。Router 只把它放进统一命令，legacy topic 由 dispatcher 发布。
bool joint_target_valid
sensor_msgs/JointState joint_target
