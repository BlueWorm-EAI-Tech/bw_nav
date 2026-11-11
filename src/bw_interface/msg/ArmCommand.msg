# 统一手臂命令。Router 之后只输出本消息，后级 resolver 再转成 14 轴 JointState。
uint8 COMMAND_NONE=0
uint8 COMMAND_JOINT=1
uint8 COMMAND_POSE_ABS=2
uint8 COMMAND_POSE_REL=3

std_msgs/Header header
uint8 command_type
string source

# COMMAND_JOINT 使用；允许只携带部分关节，resolver 会用最近目标补齐。
sensor_msgs/JointState joints

# COMMAND_POSE_ABS 使用 left_pose/right_pose；COMMAND_POSE_REL 使用 left_delta/right_delta。
bool left_valid
bool right_valid
geometry_msgs/Pose left_pose
geometry_msgs/Pose right_pose

# [dx, dy, dz, droll, dpitch, dyaw]。平移为 base/world 全局增量，旋转为末端局部增量。
float64[6] left_delta
float64[6] right_delta

# SDK 阻塞等待链路使用的命令追踪 ID。非 SDK 输入源可为空。
string command_id

# 可选运动约束。SDK 可逐条命令覆盖；Router 可按来源补默认约束。
bool motion_profile_valid
bw_interface/MotionProfile motion_profile
