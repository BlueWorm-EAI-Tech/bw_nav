# 动作文件回放时按段下发给 smoother_node 的 Ruckig 运动约束。
# mode/profile_mode 用于记录 Web 端生成策略，数值字段单位分别为 rad/s、rad/s^2、rad/s^3。
std_msgs/Header header
string mode
float64 max_velocity
float64 max_acceleration
float64 max_jerk
float64 duration_sec
