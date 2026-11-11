std_msgs/Header header

# System States (系统状态常量)
uint8 SYS_UNINITIALIZED = 0  # 未初始化
uint8 SYS_STANDBY = 1        # 待机
uint8 SYS_ACTIVE = 2         # 激活
uint8 SYS_EMERGENCY = 3      # 紧急状态
uint8 SYS_RESET = 4          # 复位状态
uint8 SYS_MENU = 5           # 菜单配置状态

# Control Sources (控制源常量)
uint8 SRC_NONE = 0
uint8 SRC_VR = 1
uint8 SRC_SDK = 2
uint8 SRC_MOTION = 3
uint8 SRC_FORCE_HANDLE = 4

# Global States
uint8 system_state           # 当前系统状态
uint8 control_source         # 当前控制源
string message               # 状态描述信息
string[] missing_devices      # 未连接的设备列表
string[] missing_nodes        # 未启动的节点列表

# Motion States Dictionary (模拟字典结构)
# 0: 静止/Static, 1: 运动/Moving
string[] motion_names        # 运动项名称 (如 "left_shoulder_pitch", "chassis", "waist" 等)
uint8[] motion_states        # 对应的运动状态 (0 或 1)
