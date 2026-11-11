# SDK 手臂命令生命周期状态。
uint8 STATUS_UNKNOWN=0
uint8 STATUS_RECEIVED=1
uint8 STATUS_ROUTED=2
uint8 STATUS_RESOLVED=3
uint8 STATUS_FAILED=4
uint8 STATUS_COMPLETED=5
uint8 STATUS_TIMEOUT=6

std_msgs/Header header
string command_id
string source
uint8 status
string message

# RESOLVED/COMPLETED/TIMEOUT 可携带最终目标关节，单位 rad。
sensor_msgs/JointState target
