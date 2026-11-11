#ifndef BW_SERIAL__LOGGER_HPP
#define BW_SERIAL__LOGGER_HPP

#include "protocol.hpp"
#include <iomanip>
#include <iostream>

namespace bw_serial {
    inline void logger_mantis_command(MantisCommand_t cmd) {
        static uint32_t counter = 0;
        if (counter++ % 20 == 0) {
            std::cout << "++++++++++++++++++ Beginning of Mantis Command ++++++++++++++++++" << std::endl;
            std::cout << "+ chassis_cmd_vel: [" << cmd.vx << ", "
                << cmd.vy << ", " << cmd.omega << "]" << std::endl;
            std::cout << "+  Head Pitch: " << cmd.head_pitch_cmd << std::endl;
            std::cout << "+  Head Yaw: " << cmd.head_yaw_cmd << std::endl;
            std::cout << "+  Pelvis Velocity: " << cmd.pelvis_velocity << std::endl;
            std::cout << "+  left arm q_pos: [" << cmd.left_shoulder_pitch_cmd << ", "
                << cmd.left_shoulder_yaw_cmd << ", "
                << cmd.left_shoulder_roll_cmd << ", "
                << cmd.left_elbow_pitch_cmd << ", "
                << cmd.left_wrist_roll_cmd << ", "
                << cmd.left_wrist_pitch_cmd << ", "
                << cmd.left_wrist_yaw_cmd << ", "
                << cmd.left_gripper_cmd << "]" << std::endl;
            std::cout << "+  right arm q_pos: [" << cmd.right_shoulder_pitch_cmd << ", "
                << cmd.right_shoulder_yaw_cmd << ", "
                << cmd.right_shoulder_roll_cmd << ", "
                << cmd.right_elbow_pitch_cmd << ", "
                << cmd.right_wrist_roll_cmd << ", "
                << cmd.right_wrist_pitch_cmd << ", "
                << cmd.right_wrist_yaw_cmd << ", "
                << cmd.right_gripper_cmd << "]" << std::endl;
            std::cout << "+  left gripper pos: " << cmd.left_gripper_cmd << std::endl;
            std::cout << "+  right gripper pos: " << cmd.right_gripper_cmd << std::endl;
            std::cout << "++++++++++++++++++ End of Mantis Command ++++++++++++++++++" << std::endl;
        }
    }

    inline void logger_mantis_feedback(MantisFeedback_t feedback) {
        static uint32_t counter = 0;
        if (counter++ % 20 == 0) {            
            std::cout << "++++++++++++++++++ Beginning of Mantis Feedback +++++++++++++++++++++" << std::endl;
            std::cout << "+  Frame Header: 0x" << std::hex << std::setfill('0') <<
                std::setw(2) << static_cast<int>(feedback.frame_header) << std::dec << std::endl;
            // 注意 frame_length 打印时强制转换为 int 类型，不然就是 uint8_t 类型，打印出来是 ASCII 字符
            std::cout << "+  Frame Length: " << static_cast<int>(feedback.frame_length) << std::endl;
            std::cout << "+  Pelvis Height: " << feedback.pelvis_height << std::endl;
            std::cout << "+  left arm q_pos: [" << feedback.left_shoulder_pitch_cur << ", "
                << feedback.left_shoulder_yaw_cur << ", "
                << feedback.left_shoulder_roll_cur << ", "
                << feedback.left_elbow_pitch_cur << ", "
                << feedback.left_wrist_roll_cur << ", "
                << feedback.left_wrist_pitch_cur << ", "
                << feedback.left_wrist_yaw_cur << ", "
                << feedback.left_gripper_cur << "]" << std::endl;
            std::cout << "+  right arm q_pos: [" << feedback.right_shoulder_pitch_cur << ", "
                << feedback.right_shoulder_yaw_cur << ", "
                << feedback.right_shoulder_roll_cur << ", "
                << feedback.right_elbow_pitch_cur << ", "
                << feedback.right_wrist_roll_cur << ", "
                << feedback.right_wrist_pitch_cur << ", "
                << feedback.right_wrist_yaw_cur << ", "
                << feedback.right_gripper_cur << "]" << std::endl;
            std::cout << "+  head q_pos: [" << feedback.head_pitch_cur << ", "
                << feedback.head_yaw_cur << "]" << std::endl;
            std::cout << "+  Frame Tail: 0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(feedback.frame_tail) << std::dec << std::endl;
            std::cout << "++++++++++++++++++++ End of Mantis Feedback +++++++++++++++++++++" << std::endl;
        }
    }

    inline void logger_mantis_feedback_v3(const MantisFeedbackV3_t &feedback) {
        static uint32_t counter = 0;
        if (counter++ % 20 == 0) {
            std::cout << "++++++++++++++++++ Beginning of Mantis Feedback V3 +++++++++++++++++++++" << std::endl;
            std::cout << "+  Frame Type: 0x" << std::hex << std::setfill('0')
                << std::setw(2) << static_cast<int>(feedback.frame_type) << std::dec << std::endl;
            std::cout << "+  Payload Length: " << static_cast<int>(feedback.payload_length) << std::endl;
            std::cout << "+  Status Flags: [all=" << static_cast<int>(feedback.status_flags)
                << ", left_arm=" << static_cast<int>(feedback.left_arm_status_flags)
                << ", right_arm=" << static_cast<int>(feedback.right_arm_status_flags)
                << ", chassis=" << static_cast<int>(feedback.chassis_status_flags) << "]" << std::endl;
            std::cout << "+  Chassis Vel: [" << feedback.chassis_vx << ", "
                << feedback.chassis_vy << ", " << feedback.chassis_omega << "]" << std::endl;
            std::cout << "+  Odom: [" << feedback.odom_x << ", " << feedback.odom_y
                << ", heading=" << feedback.robot_heading
                << ", heading_vel=" << feedback.robot_heading_vel << "]" << std::endl;
            std::cout << "+  Pelvis: [height=" << feedback.pelvis_height
                << ", velocity=" << feedback.pelvis_velocity << "]" << std::endl;
            std::cout << "+  Left arm q_pos: [" << feedback.left_joint_pos_0 << ", " << feedback.left_joint_pos_1
                << ", " << feedback.left_joint_pos_2 << ", " << feedback.left_joint_pos_3
                << ", " << feedback.left_joint_pos_4 << ", " << feedback.left_joint_pos_5
                << ", " << feedback.left_joint_pos_6 << ", " << feedback.left_joint_pos_7 << "]" << std::endl;
            std::cout << "+  Right arm q_pos: [" << feedback.right_joint_pos_0 << ", " << feedback.right_joint_pos_1
                << ", " << feedback.right_joint_pos_2 << ", " << feedback.right_joint_pos_3
                << ", " << feedback.right_joint_pos_4 << ", " << feedback.right_joint_pos_5
                << ", " << feedback.right_joint_pos_6 << ", " << feedback.right_joint_pos_7 << "]" << std::endl;
            std::cout << "+  Waist/Head: [" << feedback.waist_pos << ", "
                << feedback.head_yaw_pos << ", " << feedback.head_pitch_pos << "]" << std::endl;
            std::cout << "+  Left arm q_vel: [" << feedback.left_joint_vel_0 << ", " << feedback.left_joint_vel_1
                << ", " << feedback.left_joint_vel_2 << ", " << feedback.left_joint_vel_3
                << ", " << feedback.left_joint_vel_4 << ", " << feedback.left_joint_vel_5
                << ", " << feedback.left_joint_vel_6 << ", " << feedback.left_joint_vel_7 << "]" << std::endl;
            std::cout << "+  Right arm q_vel: [" << feedback.right_joint_vel_0 << ", " << feedback.right_joint_vel_1
                << ", " << feedback.right_joint_vel_2 << ", " << feedback.right_joint_vel_3
                << ", " << feedback.right_joint_vel_4 << ", " << feedback.right_joint_vel_5
                << ", " << feedback.right_joint_vel_6 << ", " << feedback.right_joint_vel_7 << "]" << std::endl;
            std::cout << "+  Waist Vel: " << feedback.waist_vel << std::endl;
            std::cout << "+  Left arm torque: [" << feedback.left_joint_torque_0 << ", " << feedback.left_joint_torque_1
                << ", " << feedback.left_joint_torque_2 << ", " << feedback.left_joint_torque_3
                << ", " << feedback.left_joint_torque_4 << ", " << feedback.left_joint_torque_5
                << ", " << feedback.left_joint_torque_6 << ", " << feedback.left_joint_torque_7 << "]" << std::endl;
            std::cout << "+  Right arm torque: [" << feedback.right_joint_torque_0 << ", " << feedback.right_joint_torque_1
                << ", " << feedback.right_joint_torque_2 << ", " << feedback.right_joint_torque_3
                << ", " << feedback.right_joint_torque_4 << ", " << feedback.right_joint_torque_5
                << ", " << feedback.right_joint_torque_6 << ", " << feedback.right_joint_torque_7 << "]" << std::endl;
            std::cout << "++++++++++++++++++++ End of Mantis Feedback V3 +++++++++++++++++++++" << std::endl;
        }
    }
}

#endif
