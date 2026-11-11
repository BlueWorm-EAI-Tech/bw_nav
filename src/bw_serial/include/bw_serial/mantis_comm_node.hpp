#ifndef BW_SERIAL__MANTIS_COMM_NODE_HPP_
#define BW_SERIAL__MANTIS_COMM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <bw_interface/msg/system_status.hpp>
#include <serial_driver/serial_driver.hpp>
#include "bw_serial/msg/mantis_command_v2.hpp"
#include "bw_serial/msg/mantis_feedback_v2.hpp"
#include "bw_serial/srv/set_ctrl_src.hpp"

// Include local headers.
#include "protocol.hpp"
#include "comm_utils.hpp"
#include "logger.hpp"
#include "bw_serial/pelvis_jog_controller.hpp"
#include "bw_serial/waist_jog_controller.hpp"
#include "bw_serial/v3_waist_safety.hpp"
#include "bw_serial/chassis_command_watchdog.hpp"

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <mutex>
#include <chrono>
#include <atomic>

namespace bw_serial
{

class MantisCommNode : public rclcpp::Node
{
public:
    explicit MantisCommNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MantisCommNode() override = default;

private:
    enum class MantisProtocolVersion : uint8_t
    {
        V1 = 1,
        V2 = 2,
        V3 = 3,
        V3_CHASSIS = 4,
    };

    enum class V3HardwareShape : uint8_t
    {
        UNKNOWN = 0,
        FULL_BODY = 1,
        CHASSIS_ONLY = 2,
    };

    void transmit_timer_callback();
    void head_pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void joint_solution_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void chassis_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void gripper_pos_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void pelvis_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void pelvis_height_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void pelvis_max_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void waist_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void waist_angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void waist_max_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void system_status_callback(const bw_interface::msg::SystemStatus::SharedPtr msg);
    void mantis_command_v2_callback(const bw_serial::msg::MantisCommandV2::SharedPtr msg);
    
    void async_receive_message();
    void publish_joint_states_fdb();
    void publish_force_fdb();
    void publish_wave_hand_switch();
    void publish_mantis_feedback_v2();
    void publish_available_arms(uint8_t mask);
    void handle_set_ctrl_src(
        const std::shared_ptr<bw_serial::srv::SetCtrlSrc::Request> request,
        const std::shared_ptr<bw_serial::srv::SetCtrlSrc::Response> response);
    std::vector<uint8_t> encode_mantis_command() const;
    bool decode_mantis_feedback(const std::vector<uint8_t> & frame);
    bool is_v3_chassis_protocol() const;
    bool is_v3_auto_detection_pending() const;
    void configure_v3_chassis_joint_state_names();
    void switch_to_v3_chassis_mode();
    bool is_lower_body_powered() const;
    // 收发帧解包内容打印：按 serial_frame_log_interval_sec 节流
    bool serial_frame_log_due(std::chrono::steady_clock::time_point & last_log_time) const;
    void log_tx_v3_frame(const MantisCommandV3_t & command);
    void log_tx_v3_chassis_frame(const MantisCommandV3Chassis_t & command);
    void log_rx_v3_frame(const MantisFeedbackV3_t & feedback);
    void log_rx_v3_chassis_frame(const MantisFeedbackV3Chassis_t & feedback);
    void stage_v3_control_enable(uint8_t requested_flag);
    void start_pelvis_startup_initialization();
    PelvisJogOutput update_pelvis_command_output(double dt_sec);
    void update_latest_pelvis_feedback();
    bool should_publish_v3_ros_feedback();

    // communication configuration
    MantisCommand_t mantis_command_{};
    MantisFeedback_t mamtis_feedback_{};
    MantisCommandV2_t mantis_command_v2_{};
    MantisFeedbackV2_t mantis_feedback_v2_{};
    MantisCommandV3_t mantis_command_v3_{};
    MantisFeedbackV3_t mantis_feedback_v3_{};
    MantisCommandV3Chassis_t mantis_command_v3_chassis_{};
    MantisFeedbackV3Chassis_t mantis_feedback_v3_chassis_{};
    // usart configuration
    std::string port_name_;
    int baud_rate_;
    int publish_frequency_;
    double v3_feedback_publish_frequency_hz_{250.0};
    std::chrono::steady_clock::time_point last_v3_feedback_publish_time_{};
    std::atomic<MantisProtocolVersion> protocol_version_{MantisProtocolVersion::V1};
    std::atomic<V3HardwareShape> v3_hardware_shape_{V3HardwareShape::UNKNOWN};
    bool auto_detect_hardware_shape_{false};
    uint64_t v3_auto_probe_counter_{0};
    std::string protocol_version_str;
    std::string robot_version_;

    // ros configuration
    rclcpp::Service<bw_serial::srv::SetCtrlSrc>::SharedPtr teleop_flag_server_;
    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
    rclcpp::TimerBase::SharedPtr transmit_timer_;
    // 发送定时器单独的回调组：配合 MultiThreadedExecutor，让 500Hz 串口发送
    // 不被其它订阅回调(smooth 500Hz、反馈解析等)在单线程里饿死，消除发送抖动
    rclcpp::CallbackGroup::SharedPtr transmit_cb_group_;
    // 保护命令结构体组帧(发送线程) 与 关节回调写入(订阅线程) 之间的并发；
    // 仅在组帧期间持有，阻塞式串口发送在锁外执行
    std::mutex command_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr head_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sol_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pelvis_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pelvis_height_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pelvis_max_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr waist_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr waist_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr waist_max_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wave_hand_switch_sub_;
    rclcpp::Subscription<bw_interface::msg::SystemStatus>::SharedPtr system_status_sub_;
    rclcpp::Subscription<bw_serial::msg::MantisCommandV2>::SharedPtr mantis_cmd_v2_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fdb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fdb_pub_V2_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_fdb_pub_V3_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr force_fdb_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wave_hand_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr available_arms_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr serial_tx_debug_pub_;
    bool serial_tx_debug_enabled_{false};
    rclcpp::Publisher<bw_serial::msg::MantisFeedbackV2>::SharedPtr mantis_fdb_v2_pub_;
    
    sensor_msgs::msg::JointState joint_state_fdb_;
    sensor_msgs::msg::JointState joint_state_fdb_V2;
    sensor_msgs::msg::JointState joint_state_fdb_V3;
    sensor_msgs::msg::JointState force_fdb;
    bw_serial::msg::MantisCommandV2 mantis_cmd_v2_msg_;
    bw_serial::msg::MantisFeedbackV2 mantis_fdb_v2_msg_;
    bool has_mantis_cmd_v2_;
    rclcpp::Time wave_hand_block_until_time_;
    std::vector<uint8_t> transmit_data_buffer;
    std::vector<uint8_t> receive_data_buffer;
    std::atomic<uint64_t> v3_rx_callback_count_{0};
    std::atomic<uint64_t> v3_rx_byte_count_{0};
    std::atomic<uint64_t> v3_rx_frame_count_{0};

    rclcpp::Time last_joint_stamp_;
    std::chrono::steady_clock::time_point last_joint_receive_time_{};
    bool has_joint_command_{false};
    double joint_command_timeout_sec_{0.2};
    uint8_t requested_v3_control_flag_{0};
    bool v3_arm_enable_awaiting_fresh_command_{false};
    rclcpp::Time v3_arm_enable_request_time_{0, 0, RCL_ROS_TIME};
    bool v3_arm_enable_precharge_active_{false};
    std::chrono::steady_clock::time_point v3_arm_enable_precharge_deadline_{};
    bool arm_watchdog_tripped_{false};
    // 底盘速度指令看门狗：nav2 停止发布 cmd_vel 后必须主动清零，否则底盘会保持最后速度
    ChassisCommandWatchdog chassis_cmd_vel_watchdog_{0.25};
    double chassis_cmd_vel_timeout_sec_{0.25};
    bool chassis_cmd_vel_timeout_reported_{false};
    bool has_logged_v3_control_flag_{false};
    uint8_t last_logged_v3_control_flag_{0};
    // 收发帧解包内容打印：间隔 <=0 表示逐帧打印
    bool serial_frame_log_enabled_{false};
    double serial_frame_log_interval_sec_{1.0};
    std::chrono::steady_clock::time_point last_serial_tx_log_time_{};
    std::chrono::steady_clock::time_point last_serial_rx_log_time_{};
    std::atomic<uint64_t> serial_rx_reject_count_{0};
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr serial_frame_log_callback_handle_;
    bool has_logged_v3_feedback_status_{false};
    uint8_t last_logged_v3_status_flags_{0};
    uint8_t last_logged_v3_left_arm_status_flags_{0};
    uint8_t last_logged_v3_right_arm_status_flags_{0};
    uint8_t last_logged_v3_chassis_status_flags_{0};
    uint8_t current_system_state_;
    bool left_gripper_invert_{false};
    bool right_gripper_invert_{false};
    double gripper_command_scale_{1.0};
    bool head_teleop_enabled_{true};
    bool waist_control_enabled_{true};
    float pelvis_speed_input_{0.0F};
    float pelvis_height_input_mm_{0.0F};
    float pelvis_max_velocity_input_mm_s_{200.0F};
    float v3_chassis_max_acc_x_{1.5F};
    float v3_chassis_max_acc_y_{1.5F};
    float v3_chassis_max_acc_omega_{1.5F};
    float v3_chassis_pelvis_acceleration_mm_s2_{1000.0F};
    bool has_pelvis_height_input_{false};
    bool has_pelvis_feedback_{false};
    float latest_pelvis_feedback_mm_{0.0F};
    float waist_speed_input_{0.0F};
    float waist_angle_input_deg_{0.0F};
    float waist_max_velocity_input_deg_s_{20.0F};
    bool has_waist_angle_input_{false};
    PelvisJogConfig pelvis_jog_config_{};
    PelvisJogController pelvis_jog_controller_{};
    WaistJogConfig waist_jog_config_{};
    WaistJogController waist_jog_controller_{};
    V3WaistSafetyDefaults v3_waist_safety_defaults_{};
    
    // Maps for transform configuration, assumed needed from original code logic
    // Wait, original code used global or member `head_transform` and `joint_transform`?
    // Let's check the original code again. They might be in protocol.hpp or comm_utils.hpp or defined in cpp.
    // I need to be careful about where `head_transform` and `joint_transform` come from.
};

} // namespace bw_serial

#endif // BW_SERIAL__MANTIS_COMM_NODE_HPP_
