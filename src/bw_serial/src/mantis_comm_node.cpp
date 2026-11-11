#include "bw_serial/mantis_comm_node.hpp"
#include "bw_serial/v3_frame_codec.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#define JOINT_RVIZ_CTRL_TEST 0 // RViz 控制测试标志位
#define JOINT_VR_TELEOP 1      // VR 遥操作标志位

namespace
{
constexpr uint8_t V3_ARM_CONTROL_MASK = 0x18U;
constexpr auto V3_ARM_ENABLE_PRECHARGE_DURATION = std::chrono::milliseconds(10);
// Some standalone chassis firmwares only answer while they receive a
// consecutive stream of 0x21 frames. Keep each probe shape in its own phase
// instead of alternating frame types on every tick.
constexpr uint64_t V3_AUTO_PROBE_PHASE_FRAMES = 500U;
constexpr uint64_t V3_AUTO_PROBE_TOTAL_FRAMES = V3_AUTO_PROBE_PHASE_FRAMES * 2U;

std::string format_hex_byte(uint8_t value)
{
    char buffer[8];
    std::snprintf(buffer, sizeof(buffer), "0x%02X", static_cast<unsigned>(value));
    return std::string(buffer);
}

std::string join_floats(const float * values, std::size_t count)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i != 0U)
        {
            stream << ", ";
        }
        stream << values[i];
    }
    return stream.str();
}

// 整机 V3 控制帧解包内容
std::string format_v3_command(const MantisCommandV3_t & command)
{
    const float left_pos[8] = {
        command.left_joint_pos_0, command.left_joint_pos_1, command.left_joint_pos_2,
        command.left_joint_pos_3, command.left_joint_pos_4, command.left_joint_pos_5,
        command.left_joint_pos_6, command.left_joint_pos_7};
    const float right_pos[8] = {
        command.right_joint_pos_0, command.right_joint_pos_1, command.right_joint_pos_2,
        command.right_joint_pos_3, command.right_joint_pos_4, command.right_joint_pos_5,
        command.right_joint_pos_6, command.right_joint_pos_7};

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "flag=" << format_hex_byte(command.control_flag)
           << " vx=" << command.vx << " vy=" << command.vy << " omega=" << command.omega
           << " max_acc=(" << command.max_acc_x << ", " << command.max_acc_y << ", "
           << command.max_acc_omega << ")"
           << " pelvis=" << command.pelvis_height << "mm/" << command.pelvis_velocity << "mm/s"
           << " waist=" << command.waist_pos
           << " head=(" << command.head_yaw_pos << ", " << command.head_pitch_pos << ")"
           << " left=[" << join_floats(left_pos, 8) << "]"
           << " right=[" << join_floats(right_pos, 8) << "]";
    return stream.str();
}

// 独立底盘/滑台 V3 控制帧解包内容
std::string format_v3_chassis_command(const MantisCommandV3Chassis_t & command)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "ctrl=" << format_hex_byte(command.control_flag)
           << " vx=" << command.vx << " vy=" << command.vy << " omega=" << command.omega
           << " max_acc=(" << command.max_acc_x << ", " << command.max_acc_y << ", "
           << command.max_acc_omega << ")"
           << " pelvis=" << command.pelvis_height << "mm/" << command.pelvis_max_velocity
           << "mm/s acc=" << command.pelvis_acceleration << "mm/s^2";
    return stream.str();
}

// 整机 V3 反馈帧解包内容
std::string format_v3_feedback(const MantisFeedbackV3_t & feedback)
{
    const float left_pos[8] = {
        feedback.left_joint_pos_0, feedback.left_joint_pos_1, feedback.left_joint_pos_2,
        feedback.left_joint_pos_3, feedback.left_joint_pos_4, feedback.left_joint_pos_5,
        feedback.left_joint_pos_6, feedback.left_joint_pos_7};
    const float right_pos[8] = {
        feedback.right_joint_pos_0, feedback.right_joint_pos_1, feedback.right_joint_pos_2,
        feedback.right_joint_pos_3, feedback.right_joint_pos_4, feedback.right_joint_pos_5,
        feedback.right_joint_pos_6, feedback.right_joint_pos_7};

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "status=" << format_hex_byte(feedback.status_flags)
           << " left=" << format_hex_byte(feedback.left_arm_status_flags)
           << " right=" << format_hex_byte(feedback.right_arm_status_flags)
           << " chassis=" << format_hex_byte(feedback.chassis_status_flags)
           << " vx=" << feedback.chassis_vx << " vy=" << feedback.chassis_vy
           << " omega=" << feedback.chassis_omega
           << " odom=(" << feedback.odom_x << ", " << feedback.odom_y << ")"
           << " heading=" << feedback.robot_heading << " w=" << feedback.robot_heading_vel
           << " pelvis=" << feedback.pelvis_height << "mm/" << feedback.pelvis_velocity << "mm/s"
           << " waist=" << feedback.waist_pos
           << " left_pos=[" << join_floats(left_pos, 8) << "]"
           << " right_pos=[" << join_floats(right_pos, 8) << "]";
    return stream.str();
}

// 独立底盘/滑台 V3 反馈帧解包内容
std::string format_v3_chassis_feedback(const MantisFeedbackV3Chassis_t & feedback)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "flags=" << format_hex_byte(feedback.status_flags)
           << " (power=" << ((feedback.status_flags >> 0U) & 0x01U)
           << " chassis=" << ((feedback.status_flags >> 1U) & 0x01U)
           << " pelvis=" << ((feedback.status_flags >> 2U) & 0x01U)
           << " wheels=" << format_hex_byte(static_cast<uint8_t>((feedback.status_flags >> 3U) & 0x0FU))
           << ")"
           << " vx=" << feedback.chassis_vx << " vy=" << feedback.chassis_vy
           << " omega=" << feedback.chassis_omega
           << " odom=(" << feedback.odom_x << ", " << feedback.odom_y << ")"
           << " heading=" << feedback.robot_heading << " w=" << feedback.robot_heading_vel
           << " pelvis=" << feedback.pelvis_height << "mm/" << feedback.pelvis_velocity << "mm/s";
    return stream.str();
}
}

namespace bw_serial
{

    bool MantisCommNode::serial_frame_log_due(
        std::chrono::steady_clock::time_point & last_log_time) const
    {
        if (!this->serial_frame_log_enabled_)
        {
            return false;
        }
        const auto now = std::chrono::steady_clock::now();
        // 间隔 <= 0 表示逐帧打印；首次打印不节流
        if (this->serial_frame_log_interval_sec_ > 0.0 &&
            last_log_time.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_log_time).count() <
            this->serial_frame_log_interval_sec_)
        {
            return false;
        }
        last_log_time = now;
        return true;
    }

    void MantisCommNode::log_tx_v3_frame(const MantisCommandV3_t & command)
    {
        if (!this->serial_frame_log_due(this->last_serial_tx_log_time_))
        {
            return;
        }
        RCLCPP_INFO(
            this->get_logger(), "TX V3 control 0x01/%u %s",
            static_cast<unsigned>(MANTIS_COMMAND_V3_PAYLOAD_LENGTH),
            format_v3_command(command).c_str());
    }

    void MantisCommNode::log_tx_v3_chassis_frame(const MantisCommandV3Chassis_t & command)
    {
        if (!this->serial_frame_log_due(this->last_serial_tx_log_time_))
        {
            return;
        }
        RCLCPP_INFO(
            this->get_logger(), "TX V3 chassis control 0x21/%u %s",
            static_cast<unsigned>(MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH),
            format_v3_chassis_command(command).c_str());
    }

    void MantisCommNode::log_rx_v3_frame(const MantisFeedbackV3_t & feedback)
    {
        if (!this->serial_frame_log_due(this->last_serial_rx_log_time_))
        {
            return;
        }
        RCLCPP_INFO(
            this->get_logger(), "RX V3 feedback 0x02/%u %s",
            static_cast<unsigned>(MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH),
            format_v3_feedback(feedback).c_str());
    }

    void MantisCommNode::log_rx_v3_chassis_frame(const MantisFeedbackV3Chassis_t & feedback)
    {
        if (!this->serial_frame_log_due(this->last_serial_rx_log_time_))
        {
            return;
        }
        RCLCPP_INFO(
            this->get_logger(), "RX V3 chassis feedback 0x22/%u %s",
            static_cast<unsigned>(MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH),
            format_v3_chassis_feedback(feedback).c_str());
    }

    bool MantisCommNode::is_v3_chassis_protocol() const
    {
        return this->protocol_version_ == MantisProtocolVersion::V3_CHASSIS;
    }

    bool MantisCommNode::is_v3_auto_detection_pending() const
    {
        return this->auto_detect_hardware_shape_ &&
            this->protocol_version_ == MantisProtocolVersion::V3 &&
            this->v3_hardware_shape_.load() == V3HardwareShape::UNKNOWN;
    }

    void MantisCommNode::configure_v3_chassis_joint_state_names()
    {
        this->joint_state_fdb_V3.name = {
            "pelvis_joint",
            "odom_x_joint",
            "odom_y_joint",
            "chassis_omega_joint",
            "robot_heading_joint"};
    }

    void MantisCommNode::switch_to_v3_chassis_mode()
    {
        std::lock_guard<std::mutex> command_lock(this->command_mutex_);
        this->protocol_version_ = MantisProtocolVersion::V3_CHASSIS;
        this->pelvis_jog_config_.default_height_mm = 1200.0F;
        this->pelvis_jog_config_.min_height_mm = 900.0F;
        this->pelvis_jog_config_.max_height_mm = 1500.0F;
        this->pelvis_jog_controller_ = PelvisJogController(this->pelvis_jog_config_);
        this->has_pelvis_height_input_ = false;
        this->mantis_command_v3_chassis_.control_flag =
            this->requested_v3_control_flag_ & 0x01U;
        this->mantis_command_v3_chassis_.pelvis_height =
            this->pelvis_jog_controller_.target_height_mm();
        this->mantis_command_v3_chassis_.pelvis_max_velocity =
            this->pelvis_jog_config_.max_speed_mm_s;
        this->mantis_command_v3_chassis_.max_acc_x = this->v3_chassis_max_acc_x_;
        this->mantis_command_v3_chassis_.max_acc_y = this->v3_chassis_max_acc_y_;
        this->mantis_command_v3_chassis_.max_acc_omega = this->v3_chassis_max_acc_omega_;
        this->mantis_command_v3_chassis_.pelvis_acceleration =
            this->v3_chassis_pelvis_acceleration_mm_s2_;
    }

    MantisCommNode::MantisCommNode(const rclcpp::NodeOptions &options)
        : Node("mantis_comm_node", options), wave_hand_block_until_time_(0, 0, RCL_ROS_TIME), last_joint_stamp_(0, 0, RCL_ROS_TIME), current_system_state_(0) // 初始化为 SYS_UNINITIALIZED
    {
        // Initialize MantisCommand_t defaults
        mantis_command_.frame_header = 0x1F;
        mantis_command_.frame_length = 94;
        mantis_command_.frame_tail = 0x1E;
        mantis_command_v2_.frame_header = 0x1F;
        mantis_command_v2_.frame_length = MANTIS_COMMAND_V2_SIZE;
        mantis_command_v2_.ctrl_src_flag = 0; // 默认软件关机
        mantis_command_v2_.frame_tail = 0x4E;
        mantis_command_v2_.packet1_tail = 0x1E;
        mantis_command_v2_.packet2_header = 0x2F;
        mantis_command_v2_.packet2_tail = 0x2E;
        mantis_command_v2_.packet3_header = 0x3F;
        mantis_command_v2_.packet3_tail = 0x3E;
        mantis_command_v2_.packet4_header = 0x4F;
        mantis_command_v3_.frame_type = MANTIS_COMMAND_V3_TYPE;
        mantis_command_v3_.payload_length = MANTIS_COMMAND_V3_PAYLOAD_LENGTH;
        mantis_command_v3_.control_flag = 0;
        mantis_command_v3_chassis_.frame_type = MANTIS_COMMAND_V3_CHASSIS_TYPE;
        mantis_command_v3_chassis_.payload_length = MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH;
        mantis_command_v3_chassis_.control_flag = 0;
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 12000000);
        this->declare_parameter("publish_frequency", 1000);
        this->declare_parameter("v3_feedback_publish_frequency_hz", 250.0);
        this->declare_parameter("protocol_version", "v2");
        this->declare_parameter("robot_version", "2.0");
        this->declare_parameter("left_gripper_invert", false);
        this->declare_parameter("right_gripper_invert", false);
        this->declare_parameter("gripper_command_scale", 1.0);
        this->declare_parameter("joint_command_timeout_sec", 0.2);
        this->declare_parameter("chassis_cmd_vel_timeout_sec", 0.25);
        this->declare_parameter("serial_tx_debug_enabled", false);
        this->declare_parameter("serial_frame_log_enabled", false);
        this->declare_parameter("serial_frame_log_interval_sec", 1.0);
        this->declare_parameter("auto_detect_hardware_shape", false);
        this->declare_parameter("head_teleop_enabled", true);
        this->declare_parameter("waist_control_enabled", true);
        this->declare_parameter("v3_fixed_waist_pos_deg", 0.0);
        this->declare_parameter("v3_fixed_waist_max_velocity_deg_s", 0.0);
        this->declare_parameter("v3_chassis_max_acc_x", 1.5);
        this->declare_parameter("v3_chassis_max_acc_y", 1.5);
        this->declare_parameter("v3_chassis_max_acc_omega", 1.5);
        this->declare_parameter("v3_chassis_pelvis_acceleration_mm_s2", 1000.0);
        this->get_parameter("port_name", this->port_name_);
        this->get_parameter("baud_rate", this->baud_rate_);
        this->get_parameter("publish_frequency", this->publish_frequency_);
        this->get_parameter(
            "v3_feedback_publish_frequency_hz", this->v3_feedback_publish_frequency_hz_);
        this->v3_feedback_publish_frequency_hz_ = std::clamp(
            this->v3_feedback_publish_frequency_hz_, 1.0, 1000.0);
        std::string configured_protocol_version;
        std::string configured_robot_version;
        this->get_parameter("protocol_version", configured_protocol_version);
        this->get_parameter("robot_version", configured_robot_version);
        this->robot_version_ = configured_robot_version;
        this->get_parameter("left_gripper_invert", this->left_gripper_invert_);
        this->get_parameter("right_gripper_invert", this->right_gripper_invert_);
        this->get_parameter("gripper_command_scale", this->gripper_command_scale_);
        this->get_parameter("joint_command_timeout_sec", this->joint_command_timeout_sec_);
        this->get_parameter("chassis_cmd_vel_timeout_sec", this->chassis_cmd_vel_timeout_sec_);
        this->get_parameter("serial_tx_debug_enabled", this->serial_tx_debug_enabled_);
        this->get_parameter("serial_frame_log_enabled", this->serial_frame_log_enabled_);
        this->get_parameter("serial_frame_log_interval_sec", this->serial_frame_log_interval_sec_);
        this->serial_frame_log_interval_sec_ = std::max(0.0, this->serial_frame_log_interval_sec_);
        // 支持运行时 ros2 param set 开关收发帧打印，调试时不用重启节点
        this->serial_frame_log_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & parameter : parameters)
                {
                    if (parameter.get_name() == "serial_frame_log_enabled")
                    {
                        this->serial_frame_log_enabled_ = parameter.as_bool();
                    }
                    else if (parameter.get_name() == "serial_frame_log_interval_sec")
                    {
                        this->serial_frame_log_interval_sec_ =
                            std::max(0.0, parameter.as_double());
                    }
                }
                return result;
            });
        this->get_parameter("auto_detect_hardware_shape", this->auto_detect_hardware_shape_);
        this->joint_command_timeout_sec_ = std::max(0.05, this->joint_command_timeout_sec_);
        this->chassis_cmd_vel_timeout_sec_ = std::max(0.05, this->chassis_cmd_vel_timeout_sec_);
        this->chassis_cmd_vel_watchdog_.set_timeout(this->chassis_cmd_vel_timeout_sec_);
        this->get_parameter("head_teleop_enabled", this->head_teleop_enabled_);
        this->get_parameter("waist_control_enabled", this->waist_control_enabled_);
        this->get_parameter("v3_chassis_max_acc_x", this->v3_chassis_max_acc_x_);
        this->get_parameter("v3_chassis_max_acc_y", this->v3_chassis_max_acc_y_);
        this->get_parameter("v3_chassis_max_acc_omega", this->v3_chassis_max_acc_omega_);
        this->get_parameter(
            "v3_chassis_pelvis_acceleration_mm_s2",
            this->v3_chassis_pelvis_acceleration_mm_s2_);
        this->gripper_command_scale_ = std::max(0.0, this->gripper_command_scale_);
        const PelvisJogConfig default_pelvis_config =
            default_pelvis_jog_config_for_robot_version(configured_robot_version);
        this->declare_parameter(
            "pelvis_default_height_mm",
            static_cast<double>(default_pelvis_config.default_height_mm));
        this->declare_parameter(
            "pelvis_min_height_mm",
            static_cast<double>(default_pelvis_config.min_height_mm));
        this->declare_parameter(
            "pelvis_max_height_mm",
            static_cast<double>(default_pelvis_config.max_height_mm));
        this->declare_parameter("pelvis_jog_max_speed_mm_s", 200.0);
        this->declare_parameter("pelvis_input_deadzone", 0.1);
        this->declare_parameter("pelvis_startup_init_enabled", true);
        this->declare_parameter("pelvis_startup_init_tolerance_mm", 10.0);
        this->declare_parameter("pelvis_startup_init_timeout_sec", 8.0);
        double pelvis_default_height_mm = static_cast<double>(default_pelvis_config.default_height_mm);
        double pelvis_min_height_mm = static_cast<double>(default_pelvis_config.min_height_mm);
        double pelvis_max_height_mm = static_cast<double>(default_pelvis_config.max_height_mm);
        double pelvis_jog_max_speed_mm_s = 200.0;
        double pelvis_input_deadzone = 0.1;
        bool pelvis_startup_init_enabled = true;
        double pelvis_startup_init_tolerance_mm = 10.0;
        double pelvis_startup_init_timeout_sec = 8.0;
        this->get_parameter("pelvis_default_height_mm", pelvis_default_height_mm);
        this->get_parameter("pelvis_min_height_mm", pelvis_min_height_mm);
        this->get_parameter("pelvis_max_height_mm", pelvis_max_height_mm);
        this->get_parameter("pelvis_jog_max_speed_mm_s", pelvis_jog_max_speed_mm_s);
        this->get_parameter("pelvis_input_deadzone", pelvis_input_deadzone);
        this->get_parameter("pelvis_startup_init_enabled", pelvis_startup_init_enabled);
        this->get_parameter("pelvis_startup_init_tolerance_mm", pelvis_startup_init_tolerance_mm);
        this->get_parameter("pelvis_startup_init_timeout_sec", pelvis_startup_init_timeout_sec);
        this->pelvis_jog_config_.default_height_mm = static_cast<float>(pelvis_default_height_mm);
        this->pelvis_jog_config_.min_height_mm = static_cast<float>(pelvis_min_height_mm);
        this->pelvis_jog_config_.max_height_mm = static_cast<float>(pelvis_max_height_mm);
        this->pelvis_jog_config_.max_speed_mm_s = static_cast<float>(pelvis_jog_max_speed_mm_s);
        this->pelvis_jog_config_.input_deadzone = static_cast<float>(pelvis_input_deadzone);
        this->pelvis_jog_config_.startup_init_enabled = pelvis_startup_init_enabled;
        this->pelvis_jog_config_.startup_init_tolerance_mm =
            static_cast<float>(pelvis_startup_init_tolerance_mm);
        this->pelvis_jog_config_.startup_init_timeout_sec = pelvis_startup_init_timeout_sec;
        this->pelvis_jog_controller_ = PelvisJogController(this->pelvis_jog_config_);
        double v3_fixed_waist_pos_deg = 0.0;
        double v3_fixed_waist_max_velocity_deg_s = 0.0;
        this->get_parameter("v3_fixed_waist_pos_deg", v3_fixed_waist_pos_deg);
        this->get_parameter("v3_fixed_waist_max_velocity_deg_s", v3_fixed_waist_max_velocity_deg_s);
        this->declare_parameter("waist_default_pos_deg", v3_fixed_waist_pos_deg);
        this->declare_parameter("waist_min_pos_deg", -90.0);
        this->declare_parameter("waist_max_pos_deg", 5.0);
        this->declare_parameter(
            "waist_jog_max_speed_deg_s",
            v3_fixed_waist_max_velocity_deg_s > 0.0 ? v3_fixed_waist_max_velocity_deg_s : 20.0);
        this->declare_parameter("waist_input_deadzone", 0.1);
        double waist_default_pos_deg = v3_fixed_waist_pos_deg;
        double waist_min_pos_deg = -90.0;
        double waist_max_pos_deg = 5.0;
        double waist_jog_max_speed_deg_s =
            v3_fixed_waist_max_velocity_deg_s > 0.0 ? v3_fixed_waist_max_velocity_deg_s : 20.0;
        double waist_input_deadzone = 0.1;
        this->get_parameter("waist_default_pos_deg", waist_default_pos_deg);
        this->get_parameter("waist_min_pos_deg", waist_min_pos_deg);
        this->get_parameter("waist_max_pos_deg", waist_max_pos_deg);
        this->get_parameter("waist_jog_max_speed_deg_s", waist_jog_max_speed_deg_s);
        this->get_parameter("waist_input_deadzone", waist_input_deadzone);
        this->waist_jog_config_.default_position_deg = static_cast<float>(waist_default_pos_deg);
        this->waist_jog_config_.min_position_deg = static_cast<float>(waist_min_pos_deg);
        this->waist_jog_config_.max_position_deg = static_cast<float>(waist_max_pos_deg);
        this->waist_jog_config_.max_speed_deg_s = static_cast<float>(waist_jog_max_speed_deg_s);
        this->waist_jog_config_.input_deadzone = static_cast<float>(waist_input_deadzone);
        this->waist_jog_controller_ = WaistJogController(this->waist_jog_config_);
        this->v3_waist_safety_defaults_.waist_pos_deg = static_cast<float>(v3_fixed_waist_pos_deg);
        this->v3_waist_safety_defaults_.waist_max_velocity_deg_s =
            static_cast<float>(v3_fixed_waist_max_velocity_deg_s);
        const std::string &protocol_version_str = configured_protocol_version;
        if (protocol_version_str == "v1" || protocol_version_str == "V1")
            this->protocol_version_ = MantisProtocolVersion::V1;
        else if (protocol_version_str == "v2" || protocol_version_str == "V2")
            this->protocol_version_ = MantisProtocolVersion::V2;
        else if (protocol_version_str == "v3" || protocol_version_str == "V3")
            this->protocol_version_ = MantisProtocolVersion::V3;
        else if (protocol_version_str == "v3_chassis" || protocol_version_str == "V3_CHASSIS")
            this->protocol_version_ = MantisProtocolVersion::V3_CHASSIS;
        else
        {
            this->protocol_version_ = MantisProtocolVersion::V1;
            RCLCPP_ERROR(this->get_logger(), "Invalid protocol version parameter: %s. Defaulting to v1.", protocol_version_str.c_str());
        }

        const char *protocol_name = "v1";
        if (this->protocol_version_ == MantisProtocolVersion::V2)
            protocol_name = "v2";
        else if (this->protocol_version_ == MantisProtocolVersion::V3)
            protocol_name = "v3";
        else if (this->protocol_version_ == MantisProtocolVersion::V3_CHASSIS)
            protocol_name = "v3_chassis";

        RCLCPP_INFO(
            this->get_logger(),
            "Mantis protocol version: %s",
            protocol_name);
        if (this->auto_detect_hardware_shape_ && this->protocol_version_ == MantisProtocolVersion::V3)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "V3 hardware shape auto-detection enabled (probing full-body and chassis payloads)");
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Pelvis jog config: default=%.1f mm, range=[%.1f, %.1f] mm, max_speed=%.1f mm/s, deadzone=%.2f",
            this->pelvis_jog_config_.default_height_mm,
            this->pelvis_jog_config_.min_height_mm,
            this->pelvis_jog_config_.max_height_mm,
            this->pelvis_jog_config_.max_speed_mm_s,
            this->pelvis_jog_config_.input_deadzone);
        RCLCPP_INFO(
            this->get_logger(),
            "Pelvis startup initialization: enabled=%s, tolerance=%.1f mm, timeout=%.1f s",
            this->pelvis_jog_config_.startup_init_enabled ? "true" : "false",
            this->pelvis_jog_config_.startup_init_tolerance_mm,
            this->pelvis_jog_config_.startup_init_timeout_sec);
        RCLCPP_INFO(
            this->get_logger(),
            "V3 waist safety defaults: waist_pos=%.1f deg, waist_max_vel=%.1f deg/s",
            this->v3_waist_safety_defaults_.waist_pos_deg,
            this->v3_waist_safety_defaults_.waist_max_velocity_deg_s);
        RCLCPP_INFO(
            this->get_logger(),
            "Waist jog config: default=%.1f deg, range=[%.1f, %.1f] deg, max_speed=%.1f deg/s, deadzone=%.2f",
            this->waist_jog_config_.default_position_deg,
            this->waist_jog_config_.min_position_deg,
            this->waist_jog_config_.max_position_deg,
            this->waist_jog_config_.max_speed_deg_s,
            this->waist_jog_config_.input_deadzone);
        if (this->is_v3_chassis_protocol())
        {
            this->mantis_command_v3_chassis_.max_acc_x = this->v3_chassis_max_acc_x_;
            this->mantis_command_v3_chassis_.max_acc_y = this->v3_chassis_max_acc_y_;
            this->mantis_command_v3_chassis_.max_acc_omega = this->v3_chassis_max_acc_omega_;
            this->mantis_command_v3_chassis_.pelvis_acceleration =
                this->v3_chassis_pelvis_acceleration_mm_s2_;
            RCLCPP_INFO(
                this->get_logger(),
                "Standalone chassis payload: max_acc=[%.2f, %.2f, %.2f], pelvis_acc=%.1f mm/s^2",
                this->v3_chassis_max_acc_x_,
                this->v3_chassis_max_acc_y_,
                this->v3_chassis_max_acc_omega_,
                this->v3_chassis_pelvis_acceleration_mm_s2_);
        }

        teleop_flag_server_ = this->create_service<bw_serial::srv::SetCtrlSrc>(
            "set_ctrl_src",
            std::bind(
                &MantisCommNode::handle_set_ctrl_src,
                this, std::placeholders::_1, std::placeholders::_2));
        if (this->head_teleop_enabled_)
        {
            head_pose_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "Teleop/head_pose", 10,
                std::bind(&MantisCommNode::head_pose_callback, this, std::placeholders::_1));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Head teleoperation disabled by robot profile");
        }
#if JOINT_RVIZ_CTRL_TEST // 使用 rviz 测试机器人各个关节的单独动作时，订阅 /joint_states 话题
        joint_sol_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&MantisCommNode::joint_solution_callback, this, std::placeholders::_1));
#elif JOINT_VR_TELEOP // 使用 VR 遥操作时，订阅 /Teleop/joint_angle_solution 话题
        joint_sol_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "Teleop/joint_angle_solution/smooth", 10,
            std::bind(&MantisCommNode::joint_solution_callback, this, std::placeholders::_1));
#endif
        gripper_pos_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "Teleop/gripper_pos", 10,
            std::bind(&MantisCommNode::gripper_pos_callback, this, std::placeholders::_1));
        chassis_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "Teleop/cmd_vel", 10,
            std::bind(&MantisCommNode::chassis_cmd_vel_callback, this, std::placeholders::_1));
        pelvis_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "Teleop/pelvis_speed", 10,
            std::bind(&MantisCommNode::pelvis_speed_callback, this, std::placeholders::_1));
        pelvis_height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "Teleop/pelvis_height", 10,
            std::bind(&MantisCommNode::pelvis_height_callback, this, std::placeholders::_1));
        pelvis_max_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "Teleop/pelvis_max_velocity", 10,
            std::bind(&MantisCommNode::pelvis_max_velocity_callback, this, std::placeholders::_1));
        if (this->waist_control_enabled_)
        {
            waist_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "Teleop/waist_speed", 10,
                std::bind(&MantisCommNode::waist_speed_callback, this, std::placeholders::_1));
            waist_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "Teleop/waist_angle", 10,
                std::bind(&MantisCommNode::waist_angle_callback, this, std::placeholders::_1));
            waist_max_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "Teleop/waist_max_velocity", 10,
                std::bind(&MantisCommNode::waist_max_velocity_callback, this, std::placeholders::_1));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Waist bend control disabled by robot profile");
        }
        system_status_sub_ = this->create_subscription<bw_interface::msg::SystemStatus>(
            "system/status", 10,
            std::bind(&MantisCommNode::system_status_callback, this, std::placeholders::_1));

        joint_states_fdb_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states_fdb", 10);
        joint_state_fdb_.name = {
            "pelvis_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_yaw_joint", // 修正：yaw在roll之前
            "left_shoulder_roll_joint",
            "left_elbow_pitch_joint", // 修正：添加pitch后缀
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "left_gripper_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_yaw_joint", // 修正：yaw在roll之前
            "right_shoulder_roll_joint",
            "right_elbow_pitch_joint", // 修正：添加pitch后缀
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            "right_gripper_joint",
            "head_pitch_joint",
        };
        joint_states_fdb_pub_V2_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states_fdb_V2", 10);
        joint_state_fdb_V2.name = {
            "pelvis_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_yaw_joint", // 修正：yaw在roll之前
            "left_shoulder_roll_joint",
            "left_elbow_pitch_joint", // 修正：添加pitch后缀
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "left_gripper_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_yaw_joint", // 修正：yaw在roll之前
            "right_shoulder_roll_joint",
            "right_elbow_pitch_joint", // 修正：添加pitch后缀
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            "right_gripper_joint",
            "head_pitch_joint",
            "head_yaw_joint", // V2协议额外添加的数据
            "wheel_front_left_joint",
            "wheel_front_right_joint",
            "wheel_middle_joint",
            "robot_heading_joint"};
        joint_states_fdb_pub_V3_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states_fdb_V3", 10);
        joint_state_fdb_V3.name = {
            "pelvis_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_yaw_joint",
            "left_shoulder_roll_joint",
            "left_elbow_pitch_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "left_gripper_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_yaw_joint",
            "right_shoulder_roll_joint",
            "right_elbow_pitch_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            "right_gripper_joint",
            "waist_joint",
            "head_pitch_joint",
            "head_yaw_joint",
            "odom_x_joint",
            "odom_y_joint",
            "robot_heading_joint"};
        if (this->is_v3_chassis_protocol())
            this->configure_v3_chassis_joint_state_names();

        force_fdb.name = {
            "left_force_feedback",
            "right_force_feedback"};
        force_fdb_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "force_feedback", 10);
        wave_hand_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "wave_hand_switch", 10);
        // Hardware shape is decided by the serial protocol. Use a latched,
        // reliable topic so late-starting VR/router nodes receive the result.
        // Humble does not allow transient-local durability for intra-process
        // publishers; route this one publisher through DDS like its consumers.
        rclcpp::PublisherOptions available_arms_options;
        available_arms_options.use_intra_process_comm =
            rclcpp::IntraProcessSetting::Disable;
        available_arms_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            "system/available_arms",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            available_arms_options);
        if (this->serial_tx_debug_enabled_)
        {
            serial_tx_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/debug/serial_tx", 10);
        }
        // 初始化 V2 协议中无对应话题消息的的命令消息
        this->mantis_command_v2_.max_acc_x = 1.0f;        // 最大 x 轴加速度
        this->mantis_command_v2_.max_acc_y = 1.0f;        // 最大 y 轴加速度
        this->mantis_command_v2_.max_acc_omega = 1.0f;    // 最大角速度
        this->mantis_command_v2_.pelvis_height = this->pelvis_jog_controller_.target_height_mm();
        this->mantis_command_v2_.ctrl_src_flag = 0;       // 上电信号标志位
        this->mantis_command_v2_.packet2_crc16 = 0;
        this->mantis_command_v2_.packet3_crc16 = 0;
        // V3 底盘加速度固定写入，VR 摇杆只控制最大速度命令。
        this->mantis_command_v3_.max_acc_x = 1.5f;
        this->mantis_command_v3_.max_acc_y = 1.5f;
        this->mantis_command_v3_.max_acc_omega = 1.5f;
        this->mantis_command_v3_.control_flag = 0;
        this->mantis_command_v3_.waist_pos = this->waist_jog_controller_.target_position_deg();
        this->mantis_command_v3_.waist_max_vel = 0.0F;
        this->mantis_command_v3_chassis_.pelvis_height =
            this->pelvis_jog_controller_.target_height_mm();
        this->mantis_command_v3_chassis_.pelvis_max_velocity =
            this->pelvis_jog_config_.max_speed_mm_s;
        // 初始化串口
        drivers::serial_driver::SerialPortConfig config(
            this->baud_rate_,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE);
        RCLCPP_INFO(this->get_logger(), "Serial port openning...");
        try
        {
            io_context_ = std::make_shared<drivers::common::IoContext>(1);
            // 初始化 serial_driver_
            serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
            serial_driver_->init_port(this->port_name_, config);
            serial_driver_->port()->open();

            RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
            RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
            RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
            return;
        }

        // 设置发送定时器（独立回调组 + MultiThreadedExecutor，保证发送节拍稳定）
        transmit_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        transmit_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / this->publish_frequency_)),
            std::bind(&MantisCommNode::transmit_timer_callback, this),
            transmit_cb_group_);
        RCLCPP_INFO(this->get_logger(), "wall_timer initialized successfully");
        async_receive_message(); // 进入异步接收
    }

    bool MantisCommNode::decode_mantis_feedback(const std::vector<uint8_t> &frame)
    {
        if (this->protocol_version_ == MantisProtocolVersion::V1)
        {
            if (frame.size() != MANTIS_FEEDBACK_SIZE)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid v1 feedback frame size: %zu", frame.size());
                return false;
            }
            const auto feedback_v1 = deserialize<MantisFeedback_t>(frame.data());
            uint16_t calculated_crc = 0;
            this->mamtis_feedback_.frame_header = feedback_v1.frame_header;
            this->mamtis_feedback_.frame_length = feedback_v1.frame_length;
            this->mamtis_feedback_.wave_hand_switch = feedback_v1.wave_hand_switch;
            this->mamtis_feedback_.res2 = feedback_v1.res2;
            this->mamtis_feedback_.res3 = feedback_v1.res3;
            this->mamtis_feedback_.pelvis_height = feedback_v1.pelvis_height;
            this->mamtis_feedback_.left_shoulder_pitch_cur = feedback_v1.left_shoulder_pitch_cur;
            this->mamtis_feedback_.left_shoulder_roll_cur = feedback_v1.left_shoulder_roll_cur;
            this->mamtis_feedback_.left_shoulder_yaw_cur = feedback_v1.left_shoulder_yaw_cur;
            this->mamtis_feedback_.left_elbow_pitch_cur = feedback_v1.left_elbow_pitch_cur;
            this->mamtis_feedback_.left_wrist_roll_cur = feedback_v1.left_wrist_roll_cur;
            this->mamtis_feedback_.left_wrist_pitch_cur = feedback_v1.left_wrist_pitch_cur;
            this->mamtis_feedback_.left_wrist_yaw_cur = feedback_v1.left_wrist_yaw_cur;
            this->mamtis_feedback_.left_gripper_cur = feedback_v1.left_gripper_cur;
            this->mamtis_feedback_.right_shoulder_pitch_cur = feedback_v1.right_shoulder_pitch_cur;
            this->mamtis_feedback_.right_shoulder_roll_cur = feedback_v1.right_shoulder_roll_cur;
            this->mamtis_feedback_.right_shoulder_yaw_cur = feedback_v1.right_shoulder_yaw_cur;
            this->mamtis_feedback_.right_elbow_pitch_cur = feedback_v1.right_elbow_pitch_cur;
            this->mamtis_feedback_.right_wrist_roll_cur = feedback_v1.right_wrist_roll_cur;
            this->mamtis_feedback_.right_wrist_pitch_cur = feedback_v1.right_wrist_pitch_cur;
            this->mamtis_feedback_.right_wrist_yaw_cur = feedback_v1.right_wrist_yaw_cur;
            this->mamtis_feedback_.right_gripper_cur = feedback_v1.right_gripper_cur;
            this->mamtis_feedback_.head_pitch_cur = feedback_v1.head_pitch_cur;
            this->mamtis_feedback_.head_yaw_cur = feedback_v1.head_yaw_cur;
            this->mamtis_feedback_.left_force_feedback = feedback_v1.left_force_feedback;
            this->mamtis_feedback_.right_force_feedback = feedback_v1.right_force_feedback;
            this->mamtis_feedback_.frame_tail = feedback_v1.frame_tail;
            this->mamtis_feedback_.crc16 = feedback_v1.crc16;
            crc16_update(&calculated_crc, frame.data(), frame.size() - 2);
            if (calculated_crc != feedback_v1.crc16)
            {
                RCLCPP_WARN(this->get_logger(), "V1 CRC check failed: calculated=0x%04X, received=0x%04X",
                            calculated_crc, feedback_v1.crc16);
                return false;
            }
            return true;
        }

        if (this->protocol_version_ == MantisProtocolVersion::V2)
        {
            if (frame.size() != MANTIS_FEEDBACK_V2_SIZE)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid v2 feedback frame size: %zu", frame.size());
                return false;
            }
            const auto feedback_v2 = deserialize<MantisFeedbackV2_t>(frame.data());
            if (feedback_v2.frame_header != MANTIS_FEEDBACK_V2_HEADER ||
                feedback_v2.frame_length != static_cast<uint8_t>(MANTIS_FEEDBACK_V2_SIZE) ||
                feedback_v2.packet1_tail != MANTIS_FEEDBACK_V2_PACKET1_TAIL ||
                feedback_v2.packet2_header != MANTIS_FEEDBACK_V2_PACKET2_HEADER ||
                feedback_v2.packet2_tail != MANTIS_FEEDBACK_V2_PACKET2_TAIL ||
                feedback_v2.packet3_header != MANTIS_FEEDBACK_V2_PACKET3_HEADER ||
                feedback_v2.packet3_tail != MANTIS_FEEDBACK_V2_PACKET3_TAIL ||
                feedback_v2.packet4_header != MANTIS_FEEDBACK_V2_PACKET4_HEADER ||
                feedback_v2.frame_tail != MANTIS_FEEDBACK_V2_TAIL)
            {
                RCLCPP_WARN(this->get_logger(), "Invalid v2 feedback frame markers");
                return false;
            }

            this->mantis_feedback_v2_ = feedback_v2;
            return true;
        }

        if (this->is_v3_auto_detection_pending() &&
            frame.size() == MANTIS_FEEDBACK_V3_CHASSIS_WIRE_SIZE &&
            frame[2] == MANTIS_FEEDBACK_V3_CHASSIS_TYPE)
        {
            if (!decode_v3_chassis_feedback_frame(frame, &this->mantis_feedback_v3_chassis_))
                return false;

            this->log_rx_v3_chassis_frame(this->mantis_feedback_v3_chassis_);
            this->v3_hardware_shape_.store(V3HardwareShape::CHASSIS_ONLY);
            this->switch_to_v3_chassis_mode();
            this->configure_v3_chassis_joint_state_names();
            this->publish_available_arms(0U);
            RCLCPP_INFO(
                this->get_logger(),
                "V3 hardware shape detected: chassis-only (switching to 0x21/0x22 payloads)");
        }

        if (this->is_v3_chassis_protocol())
        {
            if (!decode_v3_chassis_feedback_frame(frame, &this->mantis_feedback_v3_chassis_))
            {
                return false;
            }

            this->log_rx_v3_chassis_frame(this->mantis_feedback_v3_chassis_);

            const uint8_t status = this->mantis_feedback_v3_chassis_.status_flags;
            const bool feedback_status_changed =
                !this->has_logged_v3_feedback_status_ ||
                status != this->last_logged_v3_status_flags_;
            if (feedback_status_changed)
            {
                this->has_logged_v3_feedback_status_ = true;
                this->last_logged_v3_status_flags_ = status;
                RCLCPP_INFO(
                    this->get_logger(),
                    "V3 chassis feedback status changed: flags=0x%02X "
                    "(power=%u chassis=%u pelvis=%u wheels=0x%02X)",
                    status,
                    (status >> 0U) & 0x01U,
                    (status >> 1U) & 0x01U,
                    (status >> 2U) & 0x01U,
                    (status >> 3U) & 0x0FU);
            }
            return true;
        }

        if (frame.size() != MANTIS_FEEDBACK_V3_WIRE_SIZE)
        {
            // RCLCPP_WARN_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     5000,
            //     "Invalid v3 feedback frame size: %zu",
            //     frame.size());
            return false;
        }

        if (!decode_v3_feedback_frame(frame, &this->mantis_feedback_v3_))
        {
            // RCLCPP_WARN_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     5000,
            //     "Invalid v3 feedback wire frame");
            return false;
        }

        logger_mantis_feedback_v3(this->mantis_feedback_v3_);
        this->log_rx_v3_frame(this->mantis_feedback_v3_);

        if (this->auto_detect_hardware_shape_ &&
            this->v3_hardware_shape_.load() == V3HardwareShape::UNKNOWN)
        {
            this->v3_hardware_shape_.store(V3HardwareShape::FULL_BODY);
            this->publish_available_arms(0x03U);
            RCLCPP_INFO(
                this->get_logger(),
                "V3 hardware shape detected: full-body (keeping 0x01/0x02 payloads)");
        }

        const bool feedback_status_changed =
            !this->has_logged_v3_feedback_status_ ||
            this->mantis_feedback_v3_.status_flags != this->last_logged_v3_status_flags_ ||
            this->mantis_feedback_v3_.left_arm_status_flags !=
                this->last_logged_v3_left_arm_status_flags_ ||
            this->mantis_feedback_v3_.right_arm_status_flags !=
                this->last_logged_v3_right_arm_status_flags_ ||
            this->mantis_feedback_v3_.chassis_status_flags !=
                this->last_logged_v3_chassis_status_flags_;
        if (feedback_status_changed)
        {
            this->has_logged_v3_feedback_status_ = true;
            this->last_logged_v3_status_flags_ = this->mantis_feedback_v3_.status_flags;
            this->last_logged_v3_left_arm_status_flags_ =
                this->mantis_feedback_v3_.left_arm_status_flags;
            this->last_logged_v3_right_arm_status_flags_ =
                this->mantis_feedback_v3_.right_arm_status_flags;
            this->last_logged_v3_chassis_status_flags_ =
                this->mantis_feedback_v3_.chassis_status_flags;
            RCLCPP_INFO(
                this->get_logger(),
                "V3 feedback status changed: system=0x%02X (power=%u left=%u right=%u chassis=%u) "
                "left_motors=0x%02X right_motors=0x%02X chassis_motors=0x%02X",
                this->mantis_feedback_v3_.status_flags,
                (this->mantis_feedback_v3_.status_flags >> 0U) & 0x01U,
                (this->mantis_feedback_v3_.status_flags >> 1U) & 0x01U,
                (this->mantis_feedback_v3_.status_flags >> 2U) & 0x01U,
                (this->mantis_feedback_v3_.status_flags >> 3U) & 0x01U,
                this->mantis_feedback_v3_.left_arm_status_flags,
                this->mantis_feedback_v3_.right_arm_status_flags,
                this->mantis_feedback_v3_.chassis_status_flags);
        }
        return true;
    }

    bool MantisCommNode::is_lower_body_powered() const
    {
        if (this->protocol_version_ == MantisProtocolVersion::V2)
        {
            return this->mantis_command_v2_.ctrl_src_flag != 0;
        }
        if (this->protocol_version_ == MantisProtocolVersion::V3)
        {
            return this->mantis_command_v3_.control_flag != 0;
        }
        if (this->is_v3_chassis_protocol())
        {
            return (this->mantis_command_v3_chassis_.control_flag & 0x01U) != 0U;
        }
        return false;
    }

    void MantisCommNode::stage_v3_control_enable(uint8_t requested_flag)
    {
        const uint8_t requested_arm_bits = requested_flag & V3_ARM_CONTROL_MASK;
        const uint8_t active_arm_bits =
            this->mantis_command_v3_.control_flag & V3_ARM_CONTROL_MASK;

        if (requested_arm_bits == 0U)
        {
            this->mantis_command_v3_.control_flag = requested_flag;
            this->has_joint_command_ = false;
            this->arm_watchdog_tripped_ = false;
            this->v3_arm_enable_awaiting_fresh_command_ = false;
            this->v3_arm_enable_precharge_active_ = false;
            return;
        }

        // Changing only chassis/head permission must not pulse the already
        // enabled arms off. A fresh 14-axis command is required only when an
        // arm bit is actually being enabled from the off state.
        if (requested_arm_bits != 0U &&
            requested_arm_bits == active_arm_bits &&
            !this->arm_watchdog_tripped_)
        {
            this->mantis_command_v3_.control_flag = requested_flag;
            this->v3_arm_enable_awaiting_fresh_command_ = false;
            this->v3_arm_enable_precharge_active_ = false;
            return;
        }

        // Do not expose an intermediate software-on/arms-off flag. Retain the
        // previous lower-controller state until a fresh complete command can
        // be enabled atomically.
        this->has_joint_command_ = false;
        this->arm_watchdog_tripped_ = false;
        this->v3_arm_enable_awaiting_fresh_command_ =
            requested_arm_bits != 0U;
        this->v3_arm_enable_request_time_ = this->now();
        RCLCPP_INFO(
            this->get_logger(),
            "V3 arm enable staged: retaining control flag %u until a fresh 14-axis hold command",
            this->mantis_command_v3_.control_flag);
    }

    void MantisCommNode::start_pelvis_startup_initialization()
    {
        if (this->protocol_version_ == MantisProtocolVersion::V1)
        {
            return;
        }

        this->has_pelvis_height_input_ = false;
        this->pelvis_speed_input_ = 0.0F;
        this->pelvis_jog_controller_.reset();
        RCLCPP_INFO(
            this->get_logger(),
            "启动滑台初始化：target=%.1f mm, max_velocity=%.1f mm/s",
            this->pelvis_jog_controller_.target_height_mm(),
            this->pelvis_jog_config_.max_speed_mm_s);
    }

    PelvisJogOutput MantisCommNode::update_pelvis_command_output(double dt_sec)
    {
        if (this->pelvis_jog_controller_.startup_initialization_active() &&
            this->is_lower_body_powered())
        {
            const PelvisJogOutput startup_output =
                this->pelvis_jog_controller_.startup_initialization_step(
                    this->has_pelvis_feedback_,
                    this->latest_pelvis_feedback_mm_,
                    dt_sec);
            if (startup_output.startup_initialization_completed)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "滑台初始化结束：target=%.1f mm, feedback=%s%.1f mm",
                    startup_output.target_height_mm,
                    this->has_pelvis_feedback_ ? "" : "unknown ",
                    this->latest_pelvis_feedback_mm_);
            }
            if (startup_output.startup_initializing ||
                startup_output.startup_initialization_completed)
            {
                return startup_output;
            }
        }

        return this->pelvis_jog_controller_.step(this->pelvis_speed_input_, dt_sec);
    }

    void MantisCommNode::update_latest_pelvis_feedback()
    {
        if (this->protocol_version_ == MantisProtocolVersion::V2)
        {
            this->latest_pelvis_feedback_mm_ = this->mantis_feedback_v2_.pelvis_height;
            this->has_pelvis_feedback_ = true;
        }
        else if (this->protocol_version_ == MantisProtocolVersion::V3)
        {
            this->latest_pelvis_feedback_mm_ = this->mantis_feedback_v3_.pelvis_height;
            this->has_pelvis_feedback_ = true;
        }
        else if (this->is_v3_chassis_protocol())
        {
            this->latest_pelvis_feedback_mm_ = this->mantis_feedback_v3_chassis_.pelvis_height;
            this->has_pelvis_feedback_ = true;
        }
    }

    void MantisCommNode::transmit_timer_callback()
    {
        /**
         * @brief 上位机 -> 下位机
         * @details
         */
        try
        {
            const double dt_sec = this->publish_frequency_ > 0 ?
                1.0 / static_cast<double>(this->publish_frequency_) : 0.0;

            // 组帧期间持锁，保证读取到一致的关节/底盘等命令快照；
            // 阻塞式串口发送在解锁后执行，避免长时间持锁
            std::unique_lock<std::mutex> cmd_lock(this->command_mutex_);
            bool fallback_to_chassis = false;

            // 底盘速度指令看门狗：nav2/遥控停止发布 cmd_vel 后必须主动清零，
            // 否则底盘会一直保持最后一帧速度继续运动。
            if (this->chassis_cmd_vel_watchdog_.expired())
            {
                this->mantis_command_.vx = 0.0F;
                this->mantis_command_.vy = 0.0F;
                this->mantis_command_.omega = 0.0F;
                this->mantis_command_v2_.vx = 0.0F;
                this->mantis_command_v2_.vy = 0.0F;
                this->mantis_command_v2_.omega = 0.0F;
                this->mantis_command_v3_.vx = 0.0F;
                this->mantis_command_v3_.vy = 0.0F;
                this->mantis_command_v3_.omega = 0.0F;
                this->mantis_command_v3_chassis_.vx = 0.0F;
                this->mantis_command_v3_chassis_.vy = 0.0F;
                this->mantis_command_v3_chassis_.omega = 0.0F;
                if (!this->chassis_cmd_vel_timeout_reported_)
                {
                    this->chassis_cmd_vel_timeout_reported_ = true;
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Chassis velocity command timed out after %.3f s; forcing zero",
                        this->chassis_cmd_vel_timeout_sec_);
                }
            }

            if (this->protocol_version_ == MantisProtocolVersion::V3 && this->has_joint_command_ &&
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - this->last_joint_receive_time_).count() >
                    this->joint_command_timeout_sec_)
            {
                this->mantis_command_v3_.control_flag &=
                    static_cast<uint8_t>(~V3_ARM_CONTROL_MASK);
                if (!this->arm_watchdog_tripped_)
                {
                    this->arm_watchdog_tripped_ = true;
                    RCLCPP_ERROR(this->get_logger(), "Arm command watchdog timeout: disabling both arms");
                }
            }

            if (this->protocol_version_ == MantisProtocolVersion::V3 &&
                this->v3_arm_enable_precharge_active_ &&
                std::chrono::steady_clock::now() >= this->v3_arm_enable_precharge_deadline_)
            {
                this->v3_arm_enable_precharge_active_ = false;
                if (this->current_system_state_ != 3 &&
                    (this->requested_v3_control_flag_ & V3_ARM_CONTROL_MASK) != 0U)
                {
                    this->mantis_command_v3_.control_flag = this->requested_v3_control_flag_;
                    this->arm_watchdog_tripped_ = false;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "V3 arm precharge completed; requested arm bits enabled");
                }
            }

            if (this->current_system_state_ != 3)
            {
                if (this->protocol_version_ == MantisProtocolVersion::V1)
                {
                    this->mantis_command_.pelvis_velocity = this->pelvis_speed_input_;
                }
                else
                {
                    PelvisJogOutput pelvis_output{};
                    if (!this->has_pelvis_height_input_)
                    {
                        pelvis_output = this->update_pelvis_command_output(dt_sec);
                    }

                    if (this->protocol_version_ == MantisProtocolVersion::V2)
                    {
                        if (this->has_pelvis_height_input_)
                        {
                            this->mantis_command_v2_.pelvis_height = this->pelvis_height_input_mm_;
                            this->mantis_command_v2_.pelvis_max_velocity =
                                this->pelvis_max_velocity_input_mm_s_;
                        }
                        else
                        {
                            this->mantis_command_v2_.pelvis_height = pelvis_output.target_height_mm;
                            this->mantis_command_v2_.pelvis_max_velocity =
                                pelvis_output.max_velocity_mm_s;
                        }
                    }
                    else if (this->protocol_version_ == MantisProtocolVersion::V3)
                    {
                        if (this->has_pelvis_height_input_)
                        {
                            this->mantis_command_v3_.pelvis_height = this->pelvis_height_input_mm_;
                            this->mantis_command_v3_.pelvis_velocity =
                                this->pelvis_max_velocity_input_mm_s_;
                        }
                        else
                        {
                            this->mantis_command_v3_.pelvis_height = pelvis_output.target_height_mm;
                            this->mantis_command_v3_.pelvis_velocity =
                                pelvis_output.max_velocity_mm_s;
                        }
                    }
                    else if (this->is_v3_chassis_protocol())
                    {
                        if (this->has_pelvis_height_input_)
                        {
                            this->mantis_command_v3_chassis_.pelvis_height =
                                this->pelvis_height_input_mm_;
                            this->mantis_command_v3_chassis_.pelvis_max_velocity =
                                this->pelvis_max_velocity_input_mm_s_;
                        }
                        else
                        {
                            this->mantis_command_v3_chassis_.pelvis_height =
                                pelvis_output.target_height_mm;
                            this->mantis_command_v3_chassis_.pelvis_max_velocity =
                                pelvis_output.max_velocity_mm_s;
                        }
                        this->mantis_command_v3_chassis_.pelvis_acceleration =
                            this->v3_chassis_pelvis_acceleration_mm_s2_;
                    }
                }

                if (this->protocol_version_ == MantisProtocolVersion::V3)
                {
                    if (this->has_waist_angle_input_)
                    {
                        const float clamped_angle = std::max(
                            this->waist_jog_config_.min_position_deg,
                            std::min(this->waist_angle_input_deg_, this->waist_jog_config_.max_position_deg));
                        this->mantis_command_v3_.waist_pos = clamped_angle;
                        this->mantis_command_v3_.waist_max_vel =
                            this->waist_max_velocity_input_deg_s_;
                    }
                    else
                    {
                        const WaistJogOutput waist_output =
                            this->waist_jog_controller_.step(this->waist_speed_input_, dt_sec);
                        this->mantis_command_v3_.waist_pos = waist_output.target_position_deg;
                        this->mantis_command_v3_.waist_max_vel = waist_output.max_velocity_deg_s;
                    }
                }
            }

            // 集中安全锁：如果系统处于 EMERGENCY 状态，强制覆盖所有运动指令
            if (this->current_system_state_ == 3)
            { // SYS_EMERGENCY
                this->mantis_command_.vx = 0.0f;
                this->mantis_command_.vy = 0.0f;
                this->mantis_command_.omega = 0.0f;
                this->mantis_command_.pelvis_velocity = 0.0f;
                this->mantis_command_v2_.vx = 0.0f;
                this->mantis_command_v2_.vy = 0.0f;
                this->mantis_command_v2_.omega = 0.0f;
                this->mantis_command_v2_.pelvis_height = this->has_pelvis_height_input_
                                                             ? this->pelvis_height_input_mm_
                                                             : this->pelvis_jog_controller_.target_height_mm();
                this->mantis_command_v2_.pelvis_max_velocity = 0.0f;
                this->mantis_command_v3_.vx = 0.0f;
                this->mantis_command_v3_.vy = 0.0f;
                this->mantis_command_v3_.omega = 0.0f;
                this->mantis_command_v3_.pelvis_height = this->has_pelvis_height_input_
                                                             ? this->pelvis_height_input_mm_
                                                             : this->pelvis_jog_controller_.target_height_mm();
                this->mantis_command_v3_.pelvis_velocity = 0.0f;
                this->mantis_command_v3_chassis_.vx = 0.0f;
                this->mantis_command_v3_chassis_.vy = 0.0f;
                this->mantis_command_v3_chassis_.omega = 0.0f;
                this->mantis_command_v3_chassis_.pelvis_height = this->has_pelvis_height_input_
                    ? this->pelvis_height_input_mm_
                    : this->pelvis_jog_controller_.target_height_mm();
                this->mantis_command_v3_chassis_.pelvis_max_velocity = 0.0f;
                this->mantis_command_v3_chassis_.pelvis_acceleration =
                    this->v3_chassis_pelvis_acceleration_mm_s2_;
                this->mantis_command_v3_.waist_pos = this->has_waist_angle_input_
                                                         ? std::max(
                                                               this->waist_jog_config_.min_position_deg,
                                                               std::min(this->waist_angle_input_deg_, this->waist_jog_config_.max_position_deg))
                                                         : this->waist_jog_controller_.target_position_deg();
                this->mantis_command_v3_.waist_max_vel = 0.0f;
                // 如果需要，也可以在这里将手臂/云台指令设为当前反馈值或安全值
            }
            // logger_mantis_command(this->mantis_command_);
            // this->mantis_command_.ctrl_src_flag = 1 << 1;
            if (this->protocol_version_ == MantisProtocolVersion::V1)
            {
                // V1: 计算并填充 CRC16（结构体末尾2字节）
                this->mantis_command_.crc16 = 0;
                crc16_update(&this->mantis_command_.crc16,
                    reinterpret_cast<const uint8_t *>(&this->mantis_command_),
                    sizeof(MantisCommand_t) - 2);
                transmit_data_buffer = std::vector<uint8_t>(
                    reinterpret_cast<const uint8_t *>(&this->mantis_command_),
                    reinterpret_cast<const uint8_t *>(&this->mantis_command_) + sizeof(MantisCommand_t));
            }
            else if (this->protocol_version_ == MantisProtocolVersion::V2)
            {
                // V2: 计算并填充 CRC16
                this->mantis_command_v2_.packet2_crc16 = 0;
                this->mantis_command_v2_.packet3_crc16 = 0;
                // TODO: V2 协议 CRC 计算方式可能不同，按需补充
                transmit_data_buffer = std::vector<uint8_t>(
                    reinterpret_cast<const uint8_t *>(&this->mantis_command_v2_),
                    reinterpret_cast<const uint8_t *>(&this->mantis_command_v2_) + sizeof(MantisCommandV2_t));
            }
            else if (this->is_v3_auto_detection_pending())
            {
                // Probe one V3 shape at a time. Start with the chassis payload
                // because a standalone chassis may discard an interleaved
                // full-body frame and only answer after consecutive 0x21
                // packets. A full-body phase follows when no chassis feedback
                // arrives, preserving automatic detection for both forms.
                const uint64_t probe_index = this->v3_auto_probe_counter_++;
                const uint64_t probe_phase = probe_index / V3_AUTO_PROBE_PHASE_FRAMES;
                if ((probe_phase & 0x01U) != 0U)
                {
                    transmit_data_buffer = encode_v3_command_frame(this->mantis_command_v3_);
                    this->log_tx_v3_frame(this->mantis_command_v3_);
                }
                else
                {
                    auto chassis_probe = this->mantis_command_v3_chassis_;
                    // Probe with software power only. This is stationary and
                    // lets chassis-only firmware answer before VR connects.
                    chassis_probe.control_flag = 0x01U;
                    chassis_probe.vx = 0.0F;
                    chassis_probe.vy = 0.0F;
                    chassis_probe.omega = 0.0F;
                    chassis_probe.max_acc_x = this->v3_chassis_max_acc_x_;
                    chassis_probe.max_acc_y = this->v3_chassis_max_acc_y_;
                    chassis_probe.max_acc_omega = this->v3_chassis_max_acc_omega_;
                    chassis_probe.pelvis_height = 1200.0F;
                    chassis_probe.pelvis_max_velocity = 200.0F;
                    chassis_probe.pelvis_acceleration =
                        this->v3_chassis_pelvis_acceleration_mm_s2_;
                    transmit_data_buffer = encode_v3_chassis_command_frame(chassis_probe);
                    this->log_tx_v3_chassis_frame(chassis_probe);
                }
                fallback_to_chassis =
                    this->v3_auto_probe_counter_ >= V3_AUTO_PROBE_TOTAL_FRAMES;
            }
            else if (this->protocol_version_ == MantisProtocolVersion::V3)
            {
                if (!this->has_logged_v3_control_flag_ ||
                    this->mantis_command_v3_.control_flag != this->last_logged_v3_control_flag_)
                {
                    this->has_logged_v3_control_flag_ = true;
                    this->last_logged_v3_control_flag_ = this->mantis_command_v3_.control_flag;
                    RCLCPP_INFO(
                        this->get_logger(),
                        "V3 effective control flag changed: 0x%02X "
                        "(power=%u chassis=%u left=%u right=%u head=%u)",
                        this->mantis_command_v3_.control_flag,
                        (this->mantis_command_v3_.control_flag >> 0U) & 0x01U,
                        (this->mantis_command_v3_.control_flag >> 2U) & 0x01U,
                        (this->mantis_command_v3_.control_flag >> 3U) & 0x01U,
                        (this->mantis_command_v3_.control_flag >> 4U) & 0x01U,
                        (this->mantis_command_v3_.control_flag >> 5U) & 0x01U);
                }
                transmit_data_buffer = encode_v3_command_frame(this->mantis_command_v3_);
                this->log_tx_v3_frame(this->mantis_command_v3_);
            }
            else if (this->is_v3_chassis_protocol())
            {
                this->mantis_command_v3_chassis_.max_acc_x = this->v3_chassis_max_acc_x_;
                this->mantis_command_v3_chassis_.max_acc_y = this->v3_chassis_max_acc_y_;
                this->mantis_command_v3_chassis_.max_acc_omega = this->v3_chassis_max_acc_omega_;
                this->mantis_command_v3_chassis_.pelvis_acceleration =
                    this->v3_chassis_pelvis_acceleration_mm_s2_;
                transmit_data_buffer = encode_v3_chassis_command_frame(
                    this->mantis_command_v3_chassis_);
                this->log_tx_v3_chassis_frame(this->mantis_command_v3_chassis_);
            }
            // 组帧完成，发送前释放锁，阻塞式发送不再阻塞关节回调写入
            if (this->is_v3_auto_detection_pending())
            {
                const uint64_t probe_index = this->v3_auto_probe_counter_ == 0U
                    ? 0U : this->v3_auto_probe_counter_ - 1U;
                const char *probe_shape =
                    ((probe_index / V3_AUTO_PROBE_PHASE_FRAMES) & 0x01U) == 0U
                    ? "chassis (0x21)" : "full-body (0x01)";
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "V3 hardware probe: shape=%s tx_frames=%llu rx_callbacks=%llu "
                    "rx_bytes=%llu rx_valid_frames=%llu",
                    probe_shape,
                    static_cast<unsigned long long>(this->v3_auto_probe_counter_),
                    static_cast<unsigned long long>(this->v3_rx_callback_count_.load()),
                    static_cast<unsigned long long>(this->v3_rx_byte_count_.load()),
                    static_cast<unsigned long long>(this->v3_rx_frame_count_.load()));
            }
            cmd_lock.unlock();
            // serial_driver_->port()->send() 函数是真正的发送函数
            serial_driver_->port()->send(transmit_data_buffer);

            // Some standalone chassis firmwares do not emit a feedback frame
            // until they are fully enabled. After both probe phases, choose
            // the chassis payload so the normal VR command path can proceed.
            if (fallback_to_chassis && this->is_v3_auto_detection_pending())
            {
                this->v3_hardware_shape_.store(V3HardwareShape::CHASSIS_ONLY);
                this->switch_to_v3_chassis_mode();
                this->configure_v3_chassis_joint_state_names();
                this->publish_available_arms(0U);
                RCLCPP_WARN(
                    this->get_logger(),
                    "V3 hardware probe received no valid feedback after %llu frames; "
                    "assuming chassis-only and switching to 0x21 payloads",
                    static_cast<unsigned long long>(this->v3_auto_probe_counter_));
            }

            // 发布串口发送时间调试信息：data=[joint_stamp_sec, tx_time_sec]
            if (serial_tx_debug_pub_)
            {
                auto msg = std_msgs::msg::Float64MultiArray();
                msg.data.resize(2);
                msg.data[0] = last_joint_stamp_.seconds(); // 上一次 joint_smooth 的 header.stamp
                msg.data[1] = this->now().seconds();       // 当前串口发送时间
                serial_tx_debug_pub_->publish(msg);
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s", ex.what());
        }
    }

    void MantisCommNode::head_pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> cmd_lock(this->command_mutex_);
        auto head_transform_processor = [this, &msg](
            const std::string & name, const bool required) -> double
        {
            const auto name_it = std::find(msg->name.begin(), msg->name.end(), name);
            if (name_it != msg->name.end())
            {
                const auto index = static_cast<std::size_t>(std::distance(msg->name.begin(), name_it));
                if (index < msg->position.size())
                {
                    const auto transform_it = head_transform.find(name);
                    if (transform_it != head_transform.end())
                    {
                        return transform_it->second.first * msg->position[index] +
                            transform_it->second.second;
                    }
                }
            }
            if (required)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Head pose is missing valid joint '%s'", name.c_str());
            }
            return 0.0;
        };

        const float pitch = static_cast<float>(
            head_transform_processor("head_pitch_joint", true));
        const float yaw = static_cast<float>(
            head_transform_processor("head_yaw_joint", true));
        const float roll = static_cast<float>(
            head_transform_processor("head_roll_joint", false));
        this->mantis_command_.head_pitch_cmd = pitch;
        this->mantis_command_.head_yaw_cmd = yaw;

        this->mantis_command_v2_.head_pitch_pos = pitch;
        this->mantis_command_v2_.head_yaw_pos = yaw;
        this->mantis_command_v3_.head_pitch_pos = std::clamp(pitch, -0.785F, 0.524F);
        this->mantis_command_v3_.head_yaw_pos = std::clamp(yaw, -1.570F, 1.570F);
        // V3 Byte177 is temporarily repurposed from yaw max velocity to roll position.
        this->mantis_command_v3_.head_yaw_max_vel = std::clamp(roll, -0.349F, 0.349F);
    }

    void MantisCommNode::joint_solution_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // The standalone chassis payload has no arm fields. Keep arm IK/smoother
        // traffic out of the serial command path for this hardware shape.
        if (this->is_v3_chassis_protocol())
        {
            return;
        }

        // 与发送线程组帧互斥，保证写入的关节命令被一致地读取
        std::lock_guard<std::mutex> cmd_lock(this->command_mutex_);
        std::vector<std::string> expected_names;
        if (this->robot_version_ == "standard")
        {
            expected_names = {
                "A_left_Degree1_joint", "A_left_Degree2_joint", "A_left_Degree3_joint",
                "A_left_Degree4_joint", "A_left_Degree5_joint", "A_left_Degree6_joint",
                "A_left_Degree7_joint", "A_right_Degree1_joint", "A_right_Degree2_joint",
                "A_right_Degree3_joint", "A_right_Degree4_joint", "A_right_Degree5_joint",
                "A_right_Degree6_joint", "A_right_Degree7_joint"};
        }
        else if (msg->name.size() >= 14)
        {
            expected_names.assign(msg->name.begin(), msg->name.begin() + 14);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Rejecting joint command with fewer than 14 names");
            return;
        }
        std::map<std::string, size_t> message_index;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            message_index[msg->name[i]] = i;
        }
        for (const auto & name : expected_names)
        {
            const auto index = message_index.find(name);
            if (index == message_index.end() || index->second >= msg->position.size() ||
                !std::isfinite(msg->position[index->second]) || joint_transform.find(name) == joint_transform.end())
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Rejecting incomplete/invalid 14-axis joint command (missing %s)", name.c_str());
                return;
            }
        }

        if (this->protocol_version_ == MantisProtocolVersion::V3 &&
            this->v3_arm_enable_awaiting_fresh_command_)
        {
            const rclcpp::Time command_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
            if (command_stamp.nanoseconds() == 0 ||
                command_stamp <= this->v3_arm_enable_request_time_)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for a post-enable arm command; rejecting queued/stale smooth output");
                return;
            }
            this->v3_arm_enable_awaiting_fresh_command_ = false;
            this->mantis_command_v3_.control_flag =
                this->requested_v3_control_flag_ &
                static_cast<uint8_t>(~V3_ARM_CONTROL_MASK);
            this->v3_arm_enable_precharge_active_ = true;
            this->v3_arm_enable_precharge_deadline_ =
                std::chrono::steady_clock::now() + V3_ARM_ENABLE_PRECHARGE_DURATION;
            RCLCPP_INFO(
                this->get_logger(),
                "Fresh 14-axis hold command accepted; starting 10 ms V3 arm precharge");
        }

        this->last_joint_receive_time_ = std::chrono::steady_clock::now();
        this->has_joint_command_ = true;
        if (this->protocol_version_ == MantisProtocolVersion::V3 &&
            this->current_system_state_ != 3 &&
            !this->v3_arm_enable_awaiting_fresh_command_ &&
            !this->v3_arm_enable_precharge_active_)
        {
            this->mantis_command_v3_.control_flag = this->requested_v3_control_flag_;
            this->arm_watchdog_tripped_ = false;
        }
        // 记录关节解的 header 时间戳，供串口发送调试使用
        last_joint_stamp_ = msg->header.stamp;
        /* `/Teleop/joint_angle_solution` 顺序（按照URDF实际顺序）
         * left_shoulder_pitch_joint
         * left_shoulder_yaw_joint      // 修正：yaw在roll之前
         * left_shoulder_roll_joint
         * left_elbow_pitch_joint       // 修正：添加pitch后缀
         * left_wrist_roll_joint
         * left_wrist_pitch_joint
         * left_wrist_yaw_joint
         * right_shoulder_pitch_joint
         * right_shoulder_yaw_joint     // 修正：yaw在roll之前
         * right_shoulder_roll_joint
         * right_elbow_pitch_joint      // 修正：添加pitch后缀
         * right_wrist_roll_joint
         * right_wrist_pitch_joint
         * right_wrist_yaw_joint
         */
        auto joint_transform_processor = [this, &msg, &message_index, &expected_names](long unsigned int index)
        {
            // 关节坐标系对齐（y=kx+b)
            if (index < expected_names.size())
            {
                const auto & name = expected_names[index];
                const size_t msg_index = message_index.at(name);
                const auto transform = joint_transform.at(name);
                return transform.first * msg->position[msg_index] + transform.second;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Joint index %ld out of range", index);
                return 0.0; // 返回默认值
            }
        };

        // 处理关节速度（如果有）
        auto joint_velocity_processor = [this, &msg, &message_index, &expected_names](long unsigned int index)
        {
            // 关节坐标系对齐（y=kx+b)
            if (index < expected_names.size())
            {
                const auto & name = expected_names[index];
                const size_t msg_index = message_index.at(name);
                if (msg_index >= msg->velocity.size()) return 0.0;
                return joint_transform.at(name).first * msg->velocity[msg_index];
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Joint velocity index %ld out of range", index);
                return 0.0; // 返回默认值
            }
        };

        // 处理关节力矩（如果有）
        auto joint_effort_processor = [this, &msg, &message_index, &expected_names](long unsigned int index)
        {
            // 关节坐标系对齐（y=kx+b)
            if (index < expected_names.size())
            {
                const auto & name = expected_names[index];
                const size_t msg_index = message_index.at(name);
                if (msg_index >= msg->effort.size()) return 0.0;
                return joint_transform.at(name).first * msg->effort[msg_index];
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Joint effort index %ld out of range", index);
                return 0.0; // 返回默认值
            }
        };

        // left arm
#if JOINT_RVIZ_CTRL_TEST
        this->mantis_command_.right_shoulder_pitch_cmd = joint_transform_processor(0);
        this->mantis_command_.right_shoulder_roll_cmd = joint_transform_processor(1);
        this->mantis_command_.right_shoulder_yaw_cmd = joint_transform_processor(2);
        this->mantis_command_.right_elbow_pitch_cmd = joint_transform_processor(3); // 修正：改为elbow_pitch_cmd
        this->mantis_command_.right_wrist_roll_cmd = joint_transform_processor(4);
        this->mantis_command_.right_wrist_pitch_cmd = joint_transform_processor(5);
        this->mantis_command_.right_wrist_yaw_cmd = joint_transform_processor(6);
        // right arm
        this->mantis_command_.left_shoulder_pitch_cmd = joint_transform_processor(7);
        this->mantis_command_.left_shoulder_roll_cmd = joint_transform_processor(8);
        this->mantis_command_.left_shoulder_yaw_cmd = joint_transform_processor(9);
        this->mantis_command_.left_elbow_pitch_cmd = joint_transform_processor(10); // 修正：改为elbow_pitch_cmd
        this->mantis_command_.left_wrist_roll_cmd = joint_transform_processor(11);
        this->mantis_command_.left_wrist_pitch_cmd = joint_transform_processor(12);
        this->mantis_command_.left_wrist_yaw_cmd = joint_transform_processor(13);

        // 同时更新V2协议的关节位置
        // RVIZ测试模式下，右臂在前，左臂在后
        this->mantis_command_v2_.right_joint_pos_0 = joint_transform_processor(0);
        this->mantis_command_v2_.right_joint_pos_1 = joint_transform_processor(1);
        this->mantis_command_v2_.right_joint_pos_2 = joint_transform_processor(2);
        this->mantis_command_v2_.right_joint_pos_3 = joint_transform_processor(3);
        this->mantis_command_v2_.right_joint_pos_4 = joint_transform_processor(4);
        this->mantis_command_v2_.right_joint_pos_5 = joint_transform_processor(5);
        this->mantis_command_v2_.right_joint_pos_6 = joint_transform_processor(6);

        this->mantis_command_v2_.left_joint_pos_0 = joint_transform_processor(7);
        this->mantis_command_v2_.left_joint_pos_1 = joint_transform_processor(8);
        this->mantis_command_v2_.left_joint_pos_2 = joint_transform_processor(9);
        this->mantis_command_v2_.left_joint_pos_3 = joint_transform_processor(10);
        this->mantis_command_v2_.left_joint_pos_4 = joint_transform_processor(11);
        this->mantis_command_v2_.left_joint_pos_5 = joint_transform_processor(12);
        this->mantis_command_v2_.left_joint_pos_6 = joint_transform_processor(13);

        this->mantis_command_v3_.right_joint_pos_0 = joint_transform_processor(0);
        this->mantis_command_v3_.right_joint_pos_1 = joint_transform_processor(1);
        this->mantis_command_v3_.right_joint_pos_2 = joint_transform_processor(2);
        this->mantis_command_v3_.right_joint_pos_3 = joint_transform_processor(3);
        this->mantis_command_v3_.right_joint_pos_4 = joint_transform_processor(4);
        this->mantis_command_v3_.right_joint_pos_5 = joint_transform_processor(5);
        this->mantis_command_v3_.right_joint_pos_6 = joint_transform_processor(6);

        this->mantis_command_v3_.left_joint_pos_0 = joint_transform_processor(7);
        this->mantis_command_v3_.left_joint_pos_1 = joint_transform_processor(8);
        this->mantis_command_v3_.left_joint_pos_2 = joint_transform_processor(9);
        this->mantis_command_v3_.left_joint_pos_3 = joint_transform_processor(10);
        this->mantis_command_v3_.left_joint_pos_4 = joint_transform_processor(11);
        this->mantis_command_v3_.left_joint_pos_5 = joint_transform_processor(12);
        this->mantis_command_v3_.left_joint_pos_6 = joint_transform_processor(13);

        // 更新V2协议的关节速度
        this->mantis_command_v2_.right_joint_vel_0 = joint_velocity_processor(0);
        this->mantis_command_v2_.right_joint_vel_1 = joint_velocity_processor(1);
        this->mantis_command_v2_.right_joint_vel_2 = joint_velocity_processor(2);
        this->mantis_command_v2_.right_joint_vel_3 = joint_velocity_processor(3);
        this->mantis_command_v2_.right_joint_vel_4 = joint_velocity_processor(4);
        this->mantis_command_v2_.right_joint_vel_5 = joint_velocity_processor(5);
        this->mantis_command_v2_.right_joint_vel_6 = joint_velocity_processor(6);

        this->mantis_command_v2_.left_joint_vel_0 = joint_velocity_processor(7);
        this->mantis_command_v2_.left_joint_vel_1 = joint_velocity_processor(8);
        this->mantis_command_v2_.left_joint_vel_2 = joint_velocity_processor(9);
        this->mantis_command_v2_.left_joint_vel_3 = joint_velocity_processor(10);
        this->mantis_command_v2_.left_joint_vel_4 = joint_velocity_processor(11);
        this->mantis_command_v2_.left_joint_vel_5 = joint_velocity_processor(12);
        this->mantis_command_v2_.left_joint_vel_6 = joint_velocity_processor(13);

        this->mantis_command_v3_.right_joint_max_vel_0 = joint_velocity_processor(0);
        this->mantis_command_v3_.right_joint_max_vel_1 = joint_velocity_processor(1);
        this->mantis_command_v3_.right_joint_max_vel_2 = joint_velocity_processor(2);
        this->mantis_command_v3_.right_joint_max_vel_3 = joint_velocity_processor(3);
        this->mantis_command_v3_.right_joint_max_vel_4 = joint_velocity_processor(4);
        this->mantis_command_v3_.right_joint_max_vel_5 = joint_velocity_processor(5);
        this->mantis_command_v3_.right_joint_max_vel_6 = joint_velocity_processor(6);

        this->mantis_command_v3_.left_joint_max_vel_0 = joint_velocity_processor(7);
        this->mantis_command_v3_.left_joint_max_vel_1 = joint_velocity_processor(8);
        this->mantis_command_v3_.left_joint_max_vel_2 = joint_velocity_processor(9);
        this->mantis_command_v3_.left_joint_max_vel_3 = joint_velocity_processor(10);
        this->mantis_command_v3_.left_joint_max_vel_4 = joint_velocity_processor(11);
        this->mantis_command_v3_.left_joint_max_vel_5 = joint_velocity_processor(12);
        this->mantis_command_v3_.left_joint_max_vel_6 = joint_velocity_processor(13);

        // 更新V2协议的关节力矩
        this->mantis_command_v2_.right_joint_torque_0 = joint_effort_processor(0);
        this->mantis_command_v2_.right_joint_torque_1 = joint_effort_processor(1);
        this->mantis_command_v2_.right_joint_torque_2 = joint_effort_processor(2);
        this->mantis_command_v2_.right_joint_torque_3 = joint_effort_processor(3);
        this->mantis_command_v2_.right_joint_torque_4 = joint_effort_processor(4);
        this->mantis_command_v2_.right_joint_torque_5 = joint_effort_processor(5);
        this->mantis_command_v2_.right_joint_torque_6 = joint_effort_processor(6);

        this->mantis_command_v2_.left_joint_torque_0 = joint_effort_processor(7);
        this->mantis_command_v2_.left_joint_torque_1 = joint_effort_processor(8);
        this->mantis_command_v2_.left_joint_torque_2 = joint_effort_processor(9);
        this->mantis_command_v2_.left_joint_torque_3 = joint_effort_processor(10);
        this->mantis_command_v2_.left_joint_torque_4 = joint_effort_processor(11);
        this->mantis_command_v2_.left_joint_torque_5 = joint_effort_processor(12);
        this->mantis_command_v2_.left_joint_torque_6 = joint_effort_processor(13);

        this->mantis_command_v3_.right_joint_torque_0 = joint_effort_processor(0);
        this->mantis_command_v3_.right_joint_torque_1 = joint_effort_processor(1);
        this->mantis_command_v3_.right_joint_torque_2 = joint_effort_processor(2);
        this->mantis_command_v3_.right_joint_torque_3 = joint_effort_processor(3);
        this->mantis_command_v3_.right_joint_torque_4 = joint_effort_processor(4);
        this->mantis_command_v3_.right_joint_torque_5 = joint_effort_processor(5);
        this->mantis_command_v3_.right_joint_torque_6 = joint_effort_processor(6);

        this->mantis_command_v3_.left_joint_torque_0 = joint_effort_processor(7);
        this->mantis_command_v3_.left_joint_torque_1 = joint_effort_processor(8);
        this->mantis_command_v3_.left_joint_torque_2 = joint_effort_processor(9);
        this->mantis_command_v3_.left_joint_torque_3 = joint_effort_processor(10);
        this->mantis_command_v3_.left_joint_torque_4 = joint_effort_processor(11);
        this->mantis_command_v3_.left_joint_torque_5 = joint_effort_processor(12);
        this->mantis_command_v3_.left_joint_torque_6 = joint_effort_processor(13);

#elif JOINT_VR_TELEOP
        this->mantis_command_.left_shoulder_pitch_cmd = joint_transform_processor(0);
        this->mantis_command_.left_shoulder_yaw_cmd = joint_transform_processor(1); // 修正：yaw在roll之前
        this->mantis_command_.left_shoulder_roll_cmd = joint_transform_processor(2);
        this->mantis_command_.left_elbow_pitch_cmd = joint_transform_processor(3); // 修正：改为elbow_pitch_cmd
        this->mantis_command_.left_wrist_roll_cmd = joint_transform_processor(4);
        this->mantis_command_.left_wrist_pitch_cmd = joint_transform_processor(5);
        this->mantis_command_.left_wrist_yaw_cmd = joint_transform_processor(6);
        // right arm
        this->mantis_command_.right_shoulder_pitch_cmd = joint_transform_processor(7);
        this->mantis_command_.right_shoulder_yaw_cmd = joint_transform_processor(8); // 修正：yaw在roll之前
        this->mantis_command_.right_shoulder_roll_cmd = joint_transform_processor(9);
        this->mantis_command_.right_elbow_pitch_cmd = joint_transform_processor(10); // 修正：改为elbow_pitch_cmd
        this->mantis_command_.right_wrist_roll_cmd = joint_transform_processor(11);
        this->mantis_command_.right_wrist_pitch_cmd = joint_transform_processor(12);
        this->mantis_command_.right_wrist_yaw_cmd = joint_transform_processor(13);

        // 同时更新V2协议的关节位置
        // VR遥操作模式下，左臂在前，右臂在后
        this->mantis_command_v2_.left_joint_pos_0 = joint_transform_processor(0);
        this->mantis_command_v2_.left_joint_pos_1 = joint_transform_processor(1);
        this->mantis_command_v2_.left_joint_pos_2 = joint_transform_processor(2);
        this->mantis_command_v2_.left_joint_pos_3 = joint_transform_processor(3);
        this->mantis_command_v2_.left_joint_pos_4 = joint_transform_processor(4);
        this->mantis_command_v2_.left_joint_pos_5 = joint_transform_processor(5);
        this->mantis_command_v2_.left_joint_pos_6 = joint_transform_processor(6);
        this->mantis_command_v2_.right_joint_pos_0 = joint_transform_processor(7);
        this->mantis_command_v2_.right_joint_pos_1 = joint_transform_processor(8);
        this->mantis_command_v2_.right_joint_pos_2 = joint_transform_processor(9);
        this->mantis_command_v2_.right_joint_pos_3 = joint_transform_processor(10);
        this->mantis_command_v2_.right_joint_pos_4 = joint_transform_processor(11);
        this->mantis_command_v2_.right_joint_pos_5 = joint_transform_processor(12);
        this->mantis_command_v2_.right_joint_pos_6 = joint_transform_processor(13);

        this->mantis_command_v3_.left_joint_pos_0 = joint_transform_processor(0);
        this->mantis_command_v3_.left_joint_pos_1 = joint_transform_processor(1);
        this->mantis_command_v3_.left_joint_pos_2 = joint_transform_processor(2);
        this->mantis_command_v3_.left_joint_pos_3 = joint_transform_processor(3);
        this->mantis_command_v3_.left_joint_pos_4 = joint_transform_processor(4);
        this->mantis_command_v3_.left_joint_pos_5 = joint_transform_processor(5);
        this->mantis_command_v3_.left_joint_pos_6 = joint_transform_processor(6);
        this->mantis_command_v3_.right_joint_pos_0 = joint_transform_processor(7);
        this->mantis_command_v3_.right_joint_pos_1 = joint_transform_processor(8);
        this->mantis_command_v3_.right_joint_pos_2 = joint_transform_processor(9);
        this->mantis_command_v3_.right_joint_pos_3 = joint_transform_processor(10);
        this->mantis_command_v3_.right_joint_pos_4 = joint_transform_processor(11);
        this->mantis_command_v3_.right_joint_pos_5 = joint_transform_processor(12);
        this->mantis_command_v3_.right_joint_pos_6 = joint_transform_processor(13);
        // 更新V2协议的关节速度
        this->mantis_command_v2_.left_joint_vel_0 = joint_velocity_processor(0);
        this->mantis_command_v2_.left_joint_vel_1 = joint_velocity_processor(1);
        this->mantis_command_v2_.left_joint_vel_2 = joint_velocity_processor(2);
        this->mantis_command_v2_.left_joint_vel_3 = joint_velocity_processor(3);
        this->mantis_command_v2_.left_joint_vel_4 = joint_velocity_processor(4);
        this->mantis_command_v2_.left_joint_vel_5 = joint_velocity_processor(5);
        this->mantis_command_v2_.left_joint_vel_6 = joint_velocity_processor(6);
        this->mantis_command_v2_.right_joint_vel_0 = joint_velocity_processor(7);
        this->mantis_command_v2_.right_joint_vel_1 = joint_velocity_processor(8);
        this->mantis_command_v2_.right_joint_vel_2 = joint_velocity_processor(9);
        this->mantis_command_v2_.right_joint_vel_3 = joint_velocity_processor(10);
        this->mantis_command_v2_.right_joint_vel_4 = joint_velocity_processor(11);
        this->mantis_command_v2_.right_joint_vel_5 = joint_velocity_processor(12);
        this->mantis_command_v2_.right_joint_vel_6 = joint_velocity_processor(13);

        this->mantis_command_v3_.left_joint_max_vel_0 = joint_velocity_processor(0);
        this->mantis_command_v3_.left_joint_max_vel_1 = joint_velocity_processor(1);
        this->mantis_command_v3_.left_joint_max_vel_2 = joint_velocity_processor(2);
        this->mantis_command_v3_.left_joint_max_vel_3 = joint_velocity_processor(3);
        this->mantis_command_v3_.left_joint_max_vel_4 = joint_velocity_processor(4);
        this->mantis_command_v3_.left_joint_max_vel_5 = joint_velocity_processor(5);
        this->mantis_command_v3_.left_joint_max_vel_6 = joint_velocity_processor(6);
        this->mantis_command_v3_.right_joint_max_vel_0 = joint_velocity_processor(7);
        this->mantis_command_v3_.right_joint_max_vel_1 = joint_velocity_processor(8);
        this->mantis_command_v3_.right_joint_max_vel_2 = joint_velocity_processor(9);
        this->mantis_command_v3_.right_joint_max_vel_3 = joint_velocity_processor(10);
        this->mantis_command_v3_.right_joint_max_vel_4 = joint_velocity_processor(11);
        this->mantis_command_v3_.right_joint_max_vel_5 = joint_velocity_processor(12);
        this->mantis_command_v3_.right_joint_max_vel_6 = joint_velocity_processor(13);
        // 更新V2协议的关节力矩
        this->mantis_command_v2_.left_joint_torque_0 = joint_effort_processor(0);
        this->mantis_command_v2_.left_joint_torque_1 = joint_effort_processor(1);
        this->mantis_command_v2_.left_joint_torque_2 = joint_effort_processor(2);
        this->mantis_command_v2_.left_joint_torque_3 = joint_effort_processor(3);
        this->mantis_command_v2_.left_joint_torque_4 = joint_effort_processor(4);
        this->mantis_command_v2_.left_joint_torque_5 = joint_effort_processor(5);
        this->mantis_command_v2_.left_joint_torque_6 = joint_effort_processor(6);
        this->mantis_command_v2_.right_joint_torque_0 = joint_effort_processor(7);
        this->mantis_command_v2_.right_joint_torque_1 = joint_effort_processor(8);
        this->mantis_command_v2_.right_joint_torque_2 = joint_effort_processor(9);
        this->mantis_command_v2_.right_joint_torque_3 = joint_effort_processor(10);
        this->mantis_command_v2_.right_joint_torque_4 = joint_effort_processor(11);
        this->mantis_command_v2_.right_joint_torque_5 = joint_effort_processor(12);
        this->mantis_command_v2_.right_joint_torque_6 = joint_effort_processor(13);

        this->mantis_command_v3_.left_joint_torque_0 = joint_effort_processor(0);
        this->mantis_command_v3_.left_joint_torque_1 = joint_effort_processor(1);
        this->mantis_command_v3_.left_joint_torque_2 = joint_effort_processor(2);
        this->mantis_command_v3_.left_joint_torque_3 = joint_effort_processor(3);
        this->mantis_command_v3_.left_joint_torque_4 = joint_effort_processor(4);
        this->mantis_command_v3_.left_joint_torque_5 = joint_effort_processor(5);
        this->mantis_command_v3_.left_joint_torque_6 = joint_effort_processor(6);
        this->mantis_command_v3_.right_joint_torque_0 = joint_effort_processor(7);
        this->mantis_command_v3_.right_joint_torque_1 = joint_effort_processor(8);
        this->mantis_command_v3_.right_joint_torque_2 = joint_effort_processor(9);
        this->mantis_command_v3_.right_joint_torque_3 = joint_effort_processor(10);
        this->mantis_command_v3_.right_joint_torque_4 = joint_effort_processor(11);
        this->mantis_command_v3_.right_joint_torque_5 = joint_effort_processor(12);
        this->mantis_command_v3_.right_joint_torque_6 = joint_effort_processor(13);
#endif
    }

    void MantisCommNode::chassis_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> cmd_lock(this->command_mutex_);
        this->mantis_command_.vx = msg->linear.x;     // 前后速度
        this->mantis_command_.vy = msg->linear.y;     // 左右速度
        this->mantis_command_.omega = msg->angular.z; // 角速度

        // 同时更新V2协议的底盘速度
        this->mantis_command_v2_.vx = msg->linear.x;
        this->mantis_command_v2_.vy = msg->linear.y;
        this->mantis_command_v2_.omega = msg->angular.z;
        this->mantis_command_v3_.vx = msg->linear.x;
        this->mantis_command_v3_.vy = msg->linear.y;
        this->mantis_command_v3_.omega = msg->angular.z;
        this->mantis_command_v3_chassis_.vx = msg->linear.x;
        this->mantis_command_v3_chassis_.vy = msg->linear.y;
        this->mantis_command_v3_chassis_.omega = msg->angular.z;
        this->chassis_cmd_vel_watchdog_.observe();
        this->chassis_cmd_vel_timeout_reported_ = false;
    }

    void MantisCommNode::gripper_pos_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto map_gripper_cmd = [this](double value, bool invert) -> float
        {
            const double clamped = std::clamp(value, 0.0, 1.0);
            const double normalized = invert ? (1.0 - clamped) : clamped;
            return static_cast<float>(normalized * this->gripper_command_scale_);
        };

        const double left_raw = msg->position.size() > 0 ? msg->position[0] : 0.0;
        const double right_raw = msg->position.size() > 1 ? msg->position[1] : 0.0;
        const float left_cmd = map_gripper_cmd(left_raw, this->left_gripper_invert_);
        const float right_cmd = map_gripper_cmd(right_raw, this->right_gripper_invert_);

        this->mantis_command_.left_gripper_cmd = left_cmd;
        this->mantis_command_.right_gripper_cmd = right_cmd;

        // 同时更新V2协议的夹爪位置
        // 注意：V2协议中夹爪位置是关节位置数组中的第8个元素（索引7）
        this->mantis_command_v2_.left_joint_pos_7 = left_cmd;
        // 右夹爪对应right_joint_pos[7]
        this->mantis_command_v2_.right_joint_pos_7 = right_cmd;
        this->mantis_command_v3_.left_joint_pos_7 = left_cmd;
        this->mantis_command_v3_.right_joint_pos_7 = right_cmd;
        // 右夹爪速度对应right_joint_vel[7]
        // 更新V2协议的夹爪力矩
        // 左夹爪力矩对应left_joint_torque[7]
    }

    void MantisCommNode::pelvis_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const float previous_input = this->pelvis_speed_input_;
        this->pelvis_speed_input_ = msg->data;
        const bool previous_active = std::fabs(previous_input) > this->pelvis_jog_config_.input_deadzone;
        const bool current_active = std::fabs(this->pelvis_speed_input_) > this->pelvis_jog_config_.input_deadzone;
        const bool direction_changed =
            previous_active && current_active && ((previous_input > 0.0F) != (this->pelvis_speed_input_ > 0.0F));
        if (current_active)
        {
            this->pelvis_jog_controller_.stop_startup_initialization();
            if ((!previous_active || direction_changed) && this->has_pelvis_feedback_)
            {
                this->pelvis_jog_controller_.sync_target_height_mm(this->latest_pelvis_feedback_mm_);
            }
            this->has_pelvis_height_input_ = false;
        }
    }

    void MantisCommNode::pelvis_height_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const float requested_height_mm =
            this->pelvis_jog_config_.default_height_mm + static_cast<float>(msg->data * 1000.0F);
        this->pelvis_height_input_mm_ = std::max(
            this->pelvis_jog_config_.min_height_mm,
            std::min(requested_height_mm, this->pelvis_jog_config_.max_height_mm));
        this->pelvis_jog_controller_.stop_startup_initialization();
        this->has_pelvis_height_input_ = true;
    }

    void MantisCommNode::pelvis_max_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const float requested_velocity_mm_s = static_cast<float>(std::fabs(msg->data));
        this->pelvis_max_velocity_input_mm_s_ = std::max(
            0.0F,
            std::min(requested_velocity_mm_s, this->pelvis_jog_config_.max_speed_mm_s));
    }

    void MantisCommNode::waist_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        this->waist_speed_input_ = msg->data;
        if (std::fabs(this->waist_speed_input_) > this->waist_jog_config_.input_deadzone)
        {
            this->has_waist_angle_input_ = false;
        }
    }

    void MantisCommNode::waist_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        this->waist_angle_input_deg_ = static_cast<float>(msg->data * 180.0 / M_PI);
        this->has_waist_angle_input_ = true;
    }

    void MantisCommNode::waist_max_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const float requested_velocity_deg_s = static_cast<float>(std::fabs(msg->data));
        this->waist_max_velocity_input_deg_s_ = std::max(
            0.0F,
            std::min(requested_velocity_deg_s, this->waist_jog_config_.max_speed_deg_s));
    }

    void MantisCommNode::system_status_callback(const bw_interface::msg::SystemStatus::SharedPtr msg)
    {
        std::lock_guard<std::mutex> cmd_lock(this->command_mutex_);
        uint8_t prev_state = this->current_system_state_;
        const bool was_lower_body_powered = this->is_lower_body_powered();
        this->current_system_state_ = msg->system_state;

        // 进入 EMERGENCY 时掉电
        if (prev_state != 3 && msg->system_state == 3)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "系统进入 EMERGENCY：已发送掉电信号并拦截所有运动指令");
            this->mantis_command_.ctrl_src_flag = 0;
            this->mantis_command_v2_.ctrl_src_flag = 0;
            this->mantis_command_v3_.control_flag = 0;
            this->mantis_command_v3_chassis_.control_flag = 0;
            this->v3_arm_enable_awaiting_fresh_command_ = false;
            this->v3_arm_enable_precharge_active_ = false;
        }
        // Only an actual EMERGENCY exit may restore a previously requested
        // control source. Repeated STANDBY status messages must not defeat the
        // V3 arm command watchdog.
        else if (msg->system_state != 3 && msg->system_state != 0 &&
                 (this->protocol_version_ != MantisProtocolVersion::V3 || prev_state == 3))
        {
            uint8_t power_on_value = 0b10;
            if (this->protocol_version_ == MantisProtocolVersion::V2)
                power_on_value = 0b01;
            else if (this->protocol_version_ == MantisProtocolVersion::V3)
                // V3 必须由 VR/SDK 显式请求使能。普通系统状态变化不得自行上电。
                power_on_value = this->requested_v3_control_flag_;
            else if (this->is_v3_chassis_protocol())
                power_on_value = this->requested_v3_control_flag_ & 0x01U;
            this->mantis_command_.ctrl_src_flag = power_on_value;
            this->mantis_command_v2_.ctrl_src_flag = power_on_value;
            if (this->protocol_version_ == MantisProtocolVersion::V3)
                this->stage_v3_control_enable(power_on_value);
            else if (this->is_v3_chassis_protocol())
                this->mantis_command_v3_chassis_.control_flag = power_on_value;
            else
                this->mantis_command_v3_.control_flag = power_on_value;
        }

        if (!was_lower_body_powered && this->is_lower_body_powered())
        {
            this->start_pelvis_startup_initialization();
        }
    }

    void MantisCommNode::async_receive_message()
    {
        auto port = serial_driver_->port();

        // 设置接收回调函数
        port->async_receive(
            [this](const std::vector<uint8_t> &data, const size_t &size)
            {
                if (size > 0)
                {
                    this->v3_rx_callback_count_.fetch_add(1U, std::memory_order_relaxed);
                    this->v3_rx_byte_count_.fetch_add(size, std::memory_order_relaxed);
                    // 处理接收到的数据
                    // RCLCPP_INFO(this->get_logger(), "Received %zu bytes from serial port", size);
                    this->receive_data_buffer.insert(
                        receive_data_buffer.end(), data.begin(), data.begin() + size);
                    if (this->protocol_version_ == MantisProtocolVersion::V3 ||
                        this->is_v3_chassis_protocol())
                    {
                        const bool auto_detect_pending = this->is_v3_auto_detection_pending();
                        const uint8_t expected_frame_type = this->is_v3_chassis_protocol()
                            ? MANTIS_FEEDBACK_V3_CHASSIS_TYPE
                            : MANTIS_FEEDBACK_V3_TYPE;
                        const uint16_t expected_payload_length = this->is_v3_chassis_protocol()
                            ? MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH
                            : MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH;
                        while (receive_data_buffer.size() >= 2)
                        {
                            const uint8_t header_pattern[2] = {
                                MANTIS_V3_FRAME_HEADER_BYTE_0,
                                MANTIS_V3_FRAME_HEADER_BYTE_1};
                            auto it = std::search(
                                receive_data_buffer.begin(),
                                receive_data_buffer.end(),
                                std::begin(header_pattern),
                                std::end(header_pattern));
                            if (it == receive_data_buffer.end())
                            {
                                if (!receive_data_buffer.empty() &&
                                    receive_data_buffer.back() == MANTIS_V3_FRAME_HEADER_BYTE_0)
                                {
                                    receive_data_buffer.erase(
                                        receive_data_buffer.begin(),
                                        receive_data_buffer.end() - 1);
                                }
                                else
                                {
                                    receive_data_buffer.clear();
                                }
                                break;
                            }

                            if (it != receive_data_buffer.begin())
                            {
                                receive_data_buffer.erase(receive_data_buffer.begin(), it);
                            }

                            if (receive_data_buffer.size() < MANTIS_V3_WIRE_PREFIX_SIZE)
                            {
                                break;
                            }

                            const uint8_t frame_type = receive_data_buffer[2];
                            const auto payload_length =
                                static_cast<size_t>(receive_data_buffer[3]) |
                                (static_cast<size_t>(receive_data_buffer[4]) << 8U);
                            const bool expected_frame =
                                auto_detect_pending
                                    ? ((frame_type == MANTIS_FEEDBACK_V3_TYPE &&
                                       payload_length == MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH) ||
                                       (frame_type == MANTIS_FEEDBACK_V3_CHASSIS_TYPE &&
                                        payload_length == MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH))
                                    : (frame_type == expected_frame_type &&
                                       payload_length == expected_payload_length);
                            if (!expected_frame)
                            {
                                this->serial_rx_reject_count_.fetch_add(1U, std::memory_order_relaxed);
                                if (this->serial_frame_log_due(this->last_serial_rx_log_time_))
                                {
                                    const uint64_t rejected = this->serial_rx_reject_count_.load(
                                        std::memory_order_relaxed);
                                    if (auto_detect_pending)
                                    {
                                        RCLCPP_WARN(
                                            this->get_logger(),
                                            "RX v3 frame rejected (%llu total): type=0x%02X len=%u, "
                                            "expected type=0x02/248 or 0x22/37 during shape probing",
                                            static_cast<unsigned long long>(rejected),
                                            static_cast<unsigned>(frame_type),
                                            static_cast<unsigned>(payload_length));
                                    }
                                    else
                                    {
                                        RCLCPP_WARN(
                                            this->get_logger(),
                                            "RX v3 frame rejected (%llu total): type=0x%02X len=%u, "
                                            "expected type=0x%02X len=%u",
                                            static_cast<unsigned long long>(rejected),
                                            static_cast<unsigned>(frame_type),
                                            static_cast<unsigned>(payload_length),
                                            static_cast<unsigned>(expected_frame_type),
                                            static_cast<unsigned>(expected_payload_length));
                                    }
                                }
                                receive_data_buffer.erase(receive_data_buffer.begin());
                                continue;
                            }

                            const size_t frame_size = MANTIS_V3_WIRE_PREFIX_SIZE +
                                payload_length + MANTIS_V3_WIRE_CRC_SIZE;
                            if (receive_data_buffer.size() < frame_size)
                            {
                                break;
                            }

                            std::vector<uint8_t> frame(
                                receive_data_buffer.begin(),
                                receive_data_buffer.begin() + frame_size);
                            if (this->decode_mantis_feedback(frame))
                            {
                                this->v3_rx_frame_count_.fetch_add(1U, std::memory_order_relaxed);
                                this->update_latest_pelvis_feedback();
                                if (this->should_publish_v3_ros_feedback())
                                {
                                    this->publish_joint_states_fdb();
                                    this->publish_force_fdb();
                                    this->publish_wave_hand_switch();
                                }
                                receive_data_buffer.erase(
                                    receive_data_buffer.begin(),
                                    receive_data_buffer.begin() + frame_size);
                            }
                            else
                            {
                                receive_data_buffer.erase(receive_data_buffer.begin());
                            }
                        }
                    }
                    else
                    {
                        const size_t feedback_frame_size = this->protocol_version_ == MantisProtocolVersion::V1
                                                               ? MANTIS_FEEDBACK_SIZE
                                                               : MANTIS_FEEDBACK_V2_SIZE;
                        const uint8_t feedback_header = this->protocol_version_ == MantisProtocolVersion::V1
                                                            ? MANTIS_FEEDBACK_HEADER
                                                            : MANTIS_FEEDBACK_V2_HEADER;
                        const uint8_t feedback_tail = this->protocol_version_ == MantisProtocolVersion::V1
                                                          ? MANTIS_FEEDBACK_TAIL
                                                          : MANTIS_FEEDBACK_V2_TAIL;

                        while (receive_data_buffer.size() >= feedback_frame_size)
                        { // 为一帧长度
#if 0                 // 打印十六进制原始数据
                    std::stringstream ss;
                    ss << std::hex << std::setfill('0');
                    for (size_t i = 0; i < receive_data_buffer.size(); ++i)
                    {
                        ss << " 0x" << std::setw(2) << static_cast<int>(receive_data_buffer[i]);
                    }
                    RCLCPP_INFO(this->get_logger(), "Received %ld bytes:%s",  receive_data_buffer.size(), ss.str().c_str());
#endif
                            // 查找帧头
                            auto it = std::find(receive_data_buffer.begin(), receive_data_buffer.end(), feedback_header);
                            if (it == receive_data_buffer.end() || std::distance(it, receive_data_buffer.end()) < static_cast<int>(feedback_frame_size))
                            {
                                // 没有帧头或剩余数据不足一帧，等待下次
                                break;
                            }
                            // 检查帧尾 - V1和V2协议的帧尾位置不同
                            // V1协议: frame_tail在倒数第1字节，CRC16在倒数第2-1字节
                            // V2协议: frame_tail在倒数第1字节
                            const int tail_offset = (this->protocol_version_ == MantisProtocolVersion::V1) ? 3 : 1;
                            if (*(it + feedback_frame_size - tail_offset) == feedback_tail)
                            {
                                // 提取并处理一帧数据
                                std::vector<uint8_t> frame(it, it + feedback_frame_size);
                                if (!this->decode_mantis_feedback(frame))
                                {
                                    receive_data_buffer.erase(receive_data_buffer.begin(), it + feedback_frame_size);
                                    continue;
                                }
                                this->publish_joint_states_fdb();
                                this->publish_force_fdb(); // 添加力反馈发布
                                this->publish_wave_hand_switch();
                                // 移除已处理数据
                                receive_data_buffer.erase(receive_data_buffer.begin(), it + feedback_frame_size);
                            }
                            else
                            {
                                // 帧头后不是合法帧，丢弃该帧头，继续查找
                                receive_data_buffer.erase(receive_data_buffer.begin(), it + 1);
                            }
                        }
                    }
                }
                // 继续监听新的数据
                async_receive_message();
            });
    }

    bool MantisCommNode::should_publish_v3_ros_feedback()
    {
        const auto now = std::chrono::steady_clock::now();
        const double period_sec = 1.0 / this->v3_feedback_publish_frequency_hz_;
        if (this->last_v3_feedback_publish_time_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(
                now - this->last_v3_feedback_publish_time_).count() < period_sec)
        {
            return false;
        }
        this->last_v3_feedback_publish_time_ = now;
        return true;
    }

    void MantisCommNode::publish_available_arms(uint8_t mask)
    {
        if (!this->available_arms_pub_)
            return;
        std_msgs::msg::UInt8 message;
        message.data = static_cast<uint8_t>(mask & 0x03U);
        this->available_arms_pub_->publish(message);
    }

    void MantisCommNode::publish_joint_states_fdb()
    {
        this->update_latest_pelvis_feedback();

        if (this->protocol_version_ == MantisProtocolVersion::V2) {
            // ===== V2 协议：数据填入 joint_state_fdb_V2 =====
            this->joint_state_fdb_V2.header.stamp = this->now();
            this->joint_state_fdb_V2.header.frame_id = "mantis";

            this->joint_state_fdb_V2.position = {
                this->mantis_feedback_v2_.pelvis_height,
                this->mantis_feedback_v2_.left_joint_pos_0,
                this->mantis_feedback_v2_.left_joint_pos_1,
                this->mantis_feedback_v2_.left_joint_pos_2,
                this->mantis_feedback_v2_.left_joint_pos_3,
                this->mantis_feedback_v2_.left_joint_pos_4,
                this->mantis_feedback_v2_.left_joint_pos_5,
                this->mantis_feedback_v2_.left_joint_pos_6,
                this->mantis_feedback_v2_.left_joint_pos_7,
                this->mantis_feedback_v2_.right_joint_pos_0,
                this->mantis_feedback_v2_.right_joint_pos_1,
                this->mantis_feedback_v2_.right_joint_pos_2,
                this->mantis_feedback_v2_.right_joint_pos_3,
                this->mantis_feedback_v2_.right_joint_pos_4,
                this->mantis_feedback_v2_.right_joint_pos_5,
                this->mantis_feedback_v2_.right_joint_pos_6,
                this->mantis_feedback_v2_.right_joint_pos_7,
                this->mantis_feedback_v2_.head_pitch_pos,
                this->mantis_feedback_v2_.head_yaw_pos,
                this->mantis_feedback_v2_.wheel_front_left_enc,
                this->mantis_feedback_v2_.wheel_front_right_enc,
                this->mantis_feedback_v2_.wheel_middle_enc,
                this->mantis_feedback_v2_.robot_heading
            };

            this->joint_state_fdb_V2.velocity = {
                this->mantis_feedback_v2_.pelvis_max_velocity,
                this->mantis_feedback_v2_.left_joint_vel_0,
                this->mantis_feedback_v2_.left_joint_vel_1,
                this->mantis_feedback_v2_.left_joint_vel_2,
                this->mantis_feedback_v2_.left_joint_vel_3,
                this->mantis_feedback_v2_.left_joint_vel_4,
                this->mantis_feedback_v2_.left_joint_vel_5,
                this->mantis_feedback_v2_.left_joint_vel_6,
                this->mantis_feedback_v2_.left_joint_vel_7,
                this->mantis_feedback_v2_.right_joint_vel_0,
                this->mantis_feedback_v2_.right_joint_vel_1,
                this->mantis_feedback_v2_.right_joint_vel_2,
                this->mantis_feedback_v2_.right_joint_vel_3,
                this->mantis_feedback_v2_.right_joint_vel_4,
                this->mantis_feedback_v2_.right_joint_vel_5,
                this->mantis_feedback_v2_.right_joint_vel_6,
                this->mantis_feedback_v2_.right_joint_vel_7,
                0.0f,  // head_pitch_vel
                0.0f,  // head_yaw_vel
                this->mantis_feedback_v2_.wheel_front_left_vel,
                this->mantis_feedback_v2_.wheel_front_right_vel,
                this->mantis_feedback_v2_.wheel_middle_vel,
                this->mantis_feedback_v2_.robot_heading_vel
            };

            this->joint_state_fdb_V2.effort = {
                0.0f,
                this->mantis_feedback_v2_.left_joint_torque_0,
                this->mantis_feedback_v2_.left_joint_torque_1,
                this->mantis_feedback_v2_.left_joint_torque_2,
                this->mantis_feedback_v2_.left_joint_torque_3,
                this->mantis_feedback_v2_.left_joint_torque_4,
                this->mantis_feedback_v2_.left_joint_torque_5,
                this->mantis_feedback_v2_.left_joint_torque_6,
                this->mantis_feedback_v2_.left_joint_torque_7,
                this->mantis_feedback_v2_.right_joint_torque_0,
                this->mantis_feedback_v2_.right_joint_torque_1,
                this->mantis_feedback_v2_.right_joint_torque_2,
                this->mantis_feedback_v2_.right_joint_torque_3,
                this->mantis_feedback_v2_.right_joint_torque_4,
                this->mantis_feedback_v2_.right_joint_torque_5,
                this->mantis_feedback_v2_.right_joint_torque_6,
                this->mantis_feedback_v2_.right_joint_torque_7,
                0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f
            };

            joint_states_fdb_pub_V2_->publish(joint_state_fdb_V2);

        } else if (this->is_v3_chassis_protocol()) {
            this->joint_state_fdb_V3.header.stamp = this->now();
            this->joint_state_fdb_V3.header.frame_id = "mantis_chassis";
            this->joint_state_fdb_V3.position = {
                this->mantis_feedback_v3_chassis_.pelvis_height,
                this->mantis_feedback_v3_chassis_.odom_x,
                this->mantis_feedback_v3_chassis_.odom_y,
                0.0F,
                this->mantis_feedback_v3_chassis_.robot_heading};
            this->joint_state_fdb_V3.velocity = {
                this->mantis_feedback_v3_chassis_.pelvis_velocity,
                this->mantis_feedback_v3_chassis_.chassis_vx,
                this->mantis_feedback_v3_chassis_.chassis_vy,
                this->mantis_feedback_v3_chassis_.chassis_omega,
                this->mantis_feedback_v3_chassis_.robot_heading_vel};
            this->joint_state_fdb_V3.effort.assign(5U, 0.0F);
            joint_states_fdb_pub_V3_->publish(joint_state_fdb_V3);

        } else if (this->protocol_version_ == MantisProtocolVersion::V3) {
            this->joint_state_fdb_V3.header.stamp = this->now();
            this->joint_state_fdb_V3.header.frame_id = "mantis";

            this->joint_state_fdb_V3.position = {
                this->mantis_feedback_v3_.pelvis_height,
                this->mantis_feedback_v3_.left_joint_pos_0,
                this->mantis_feedback_v3_.left_joint_pos_1,
                this->mantis_feedback_v3_.left_joint_pos_2,
                this->mantis_feedback_v3_.left_joint_pos_3,
                this->mantis_feedback_v3_.left_joint_pos_4,
                this->mantis_feedback_v3_.left_joint_pos_5,
                this->mantis_feedback_v3_.left_joint_pos_6,
                this->mantis_feedback_v3_.left_joint_pos_7,
                this->mantis_feedback_v3_.right_joint_pos_0,
                this->mantis_feedback_v3_.right_joint_pos_1,
                this->mantis_feedback_v3_.right_joint_pos_2,
                this->mantis_feedback_v3_.right_joint_pos_3,
                this->mantis_feedback_v3_.right_joint_pos_4,
                this->mantis_feedback_v3_.right_joint_pos_5,
                this->mantis_feedback_v3_.right_joint_pos_6,
                this->mantis_feedback_v3_.right_joint_pos_7,
                this->mantis_feedback_v3_.waist_pos,
                this->mantis_feedback_v3_.head_pitch_pos,
                this->mantis_feedback_v3_.head_yaw_pos,
                this->mantis_feedback_v3_.odom_x,
                this->mantis_feedback_v3_.odom_y,
                this->mantis_feedback_v3_.robot_heading
            };

            this->joint_state_fdb_V3.velocity = {
                this->mantis_feedback_v3_.pelvis_velocity,
                this->mantis_feedback_v3_.left_joint_vel_0,
                this->mantis_feedback_v3_.left_joint_vel_1,
                this->mantis_feedback_v3_.left_joint_vel_2,
                this->mantis_feedback_v3_.left_joint_vel_3,
                this->mantis_feedback_v3_.left_joint_vel_4,
                this->mantis_feedback_v3_.left_joint_vel_5,
                this->mantis_feedback_v3_.left_joint_vel_6,
                this->mantis_feedback_v3_.left_joint_vel_7,
                this->mantis_feedback_v3_.right_joint_vel_0,
                this->mantis_feedback_v3_.right_joint_vel_1,
                this->mantis_feedback_v3_.right_joint_vel_2,
                this->mantis_feedback_v3_.right_joint_vel_3,
                this->mantis_feedback_v3_.right_joint_vel_4,
                this->mantis_feedback_v3_.right_joint_vel_5,
                this->mantis_feedback_v3_.right_joint_vel_6,
                this->mantis_feedback_v3_.right_joint_vel_7,
                this->mantis_feedback_v3_.waist_vel,
                0.0f,
                0.0f,
                this->mantis_feedback_v3_.chassis_vx,
                this->mantis_feedback_v3_.chassis_vy,
                this->mantis_feedback_v3_.robot_heading_vel
            };

            this->joint_state_fdb_V3.effort = {
                0.0f,
                this->mantis_feedback_v3_.left_joint_torque_0,
                this->mantis_feedback_v3_.left_joint_torque_1,
                this->mantis_feedback_v3_.left_joint_torque_2,
                this->mantis_feedback_v3_.left_joint_torque_3,
                this->mantis_feedback_v3_.left_joint_torque_4,
                this->mantis_feedback_v3_.left_joint_torque_5,
                this->mantis_feedback_v3_.left_joint_torque_6,
                this->mantis_feedback_v3_.left_joint_torque_7,
                this->mantis_feedback_v3_.right_joint_torque_0,
                this->mantis_feedback_v3_.right_joint_torque_1,
                this->mantis_feedback_v3_.right_joint_torque_2,
                this->mantis_feedback_v3_.right_joint_torque_3,
                this->mantis_feedback_v3_.right_joint_torque_4,
                this->mantis_feedback_v3_.right_joint_torque_5,
                this->mantis_feedback_v3_.right_joint_torque_6,
                this->mantis_feedback_v3_.right_joint_torque_7,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            };

            joint_states_fdb_pub_V3_->publish(joint_state_fdb_V3);

        } else {
            // ===== V1 协议 =====
            this->joint_state_fdb_.header.stamp = this->now();
            this->joint_state_fdb_.header.frame_id = "mantis";
            this->joint_state_fdb_.position = {
                mamtis_feedback_.pelvis_height,
                mamtis_feedback_.left_shoulder_pitch_cur,
                mamtis_feedback_.left_shoulder_yaw_cur,
                mamtis_feedback_.left_shoulder_roll_cur,
                mamtis_feedback_.left_elbow_pitch_cur,
                mamtis_feedback_.left_wrist_roll_cur,
                mamtis_feedback_.left_wrist_pitch_cur,
                mamtis_feedback_.left_wrist_yaw_cur,
                mamtis_feedback_.left_gripper_cur,
                mamtis_feedback_.right_shoulder_pitch_cur,
                mamtis_feedback_.right_shoulder_yaw_cur,
                mamtis_feedback_.right_shoulder_roll_cur,
                mamtis_feedback_.right_elbow_pitch_cur,
                mamtis_feedback_.right_wrist_roll_cur,
                mamtis_feedback_.right_wrist_pitch_cur,
                mamtis_feedback_.right_wrist_yaw_cur,
                mamtis_feedback_.right_gripper_cur,
                mamtis_feedback_.head_pitch_cur,
                mamtis_feedback_.head_yaw_cur
            };
            joint_states_fdb_pub_->publish(joint_state_fdb_);
        }
    }

    void MantisCommNode::publish_force_fdb()
    {
        if (this->protocol_version_ == MantisProtocolVersion::V2)
        {
            force_fdb.effort = {
                this->mantis_feedback_v2_.left_joint_torque_7,
                this->mantis_feedback_v2_.right_joint_torque_7};
        }
        else if (this->protocol_version_ == MantisProtocolVersion::V3)
        {
            force_fdb.effort = {
                this->mantis_feedback_v3_.left_joint_torque_7,
                this->mantis_feedback_v3_.right_joint_torque_7};
        }
        else if (this->is_v3_chassis_protocol())
        {
            force_fdb.effort = {0.0F, 0.0F};
        }
        else
        {
            force_fdb.effort = {
                this->mamtis_feedback_.left_force_feedback,
                this->mamtis_feedback_.right_force_feedback};
        }
        force_fdb_pub_->publish(force_fdb); // 修正：应该发布force_fdb而不是joint_state_fdb_
    }

    void MantisCommNode::publish_wave_hand_switch()
    {
        // TODO: 摆手开关暂时屏蔽，避免干扰主流程
        return;

        // auto current_time = this->now();

        // // 检查是否在禁止发布期间（发布1后的13秒内）
        // if (wave_hand_block_until_time_.nanoseconds() > 0 && current_time < wave_hand_block_until_time_)
        // {
        //     // 在禁止期间，不发布任何摆手控制话题
        //     return;
        // }

        // // 判断逻辑：大于0.5为1，否则为0
        // bool should_enable = mamtis_feedback_.wave_hand_switch > 0.5;

        // std_msgs::msg::Bool wave_hand_msg;
        // wave_hand_msg.data = should_enable;

        // wave_hand_switch_pub_->publish(wave_hand_msg);

        // // 增加日志输出，但进行频率控制
        // static int wave_hand_log_counter = 0;
        // wave_hand_log_counter++;
        // if (wave_hand_log_counter >= 30)
        // {
        //     // RCLCPP_INFO(this->get_logger(), "Wave hand switch: %s (raw value: %.2f)",
        //     //             wave_hand_msg.data ? "ON" : "OFF", mamtis_feedback_.wave_hand_switch);
        //     wave_hand_log_counter = 0;
        // }

        // // 如果发布了1（启用），则设置13秒的禁止发布期
        // if (should_enable)
        // {
        //     wave_hand_block_until_time_ = current_time + rclcpp::Duration::from_nanoseconds(8000000000LL); // 13秒
        //     RCLCPP_INFO(this->get_logger(), "Wave hand activated, blocking further publications for 13 seconds");
        // }
    }

    void MantisCommNode::handle_set_ctrl_src(
        const std::shared_ptr<bw_serial::srv::SetCtrlSrc::Request> request,
        const std::shared_ptr<bw_serial::srv::SetCtrlSrc::Response> response)
    {
        std::lock_guard<std::mutex> cmd_lock(this->command_mutex_);
        const bool was_lower_body_powered = this->is_lower_body_powered();
        // 直接透传上电信号值，由调用方根据协议版本决定传入正确的值
        this->mantis_command_.ctrl_src_flag = request->value;
        this->mantis_command_v2_.ctrl_src_flag = request->value;
        this->requested_v3_control_flag_ = request->value;
        if (this->protocol_version_ == MantisProtocolVersion::V3)
            this->stage_v3_control_enable(request->value);
        else if (this->is_v3_chassis_protocol())
            this->mantis_command_v3_chassis_.control_flag = request->value & 0x01U;
        else
            this->mantis_command_v3_.control_flag = request->value;

        const bool lower_body_enable_requested =
            this->protocol_version_ == MantisProtocolVersion::V3 ?
            request->value != 0U : this->is_lower_body_powered();
        if (!was_lower_body_powered && lower_body_enable_requested)
        {
            this->start_pelvis_startup_initialization();
        }

        RCLCPP_INFO(this->get_logger(), "ctrl_src_flag set to: %ld", request->value);

        response->success = true;
        response->message = "Control Source Flag set successfully.";
    }

} // namespace bw_serial

RCLCPP_COMPONENTS_REGISTER_NODE(bw_serial::MantisCommNode)
