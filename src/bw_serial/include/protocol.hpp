#ifndef BW_SERIAL__PROTOCOL_HPP
#define BW_SERIAL__PROTOCOL_HPP

#include <cstdint>

static_assert(sizeof(float) == 4, "Protocol requires 4-byte float");

#define MANTIS_COMMAND_SIZE sizeof(MantisCommand_t)
#define MANTIS_COMMAND_HEADER 0x1F
#define MANTIS_COMMAND_TAIL 0x1E

#define MANTIS_FEEDBACK_SIZE sizeof(MantisFeedback_t)
#define MANTIS_FEEDBACK_HEADER 0x2F
#define MANTIS_FEEDBACK_TAIL 0x1E

#define MANTIS_FEEDBACK_V2_SIZE sizeof(MantisFeedbackV2_t)
#define MANTIS_FEEDBACK_V2_HEADER 0x1F
#define MANTIS_FEEDBACK_V2_PACKET1_TAIL 0x1E
#define MANTIS_FEEDBACK_V2_PACKET2_HEADER 0x2F
#define MANTIS_FEEDBACK_V2_PACKET2_TAIL 0x2E
#define MANTIS_FEEDBACK_V2_PACKET3_HEADER 0x3F
#define MANTIS_FEEDBACK_V2_PACKET3_TAIL 0x3E
#define MANTIS_FEEDBACK_V2_PACKET4_HEADER 0x4F
#define MANTIS_FEEDBACK_V2_TAIL 0x1E

#define MANTIS_COMMAND_V2_SIZE sizeof(MantisCommandV2_t)

#define MANTIS_V3_FRAME_HEADER_BYTE_0 0x55
#define MANTIS_V3_FRAME_HEADER_BYTE_1 0xAA
#define MANTIS_V3_WIRE_PREFIX_SIZE 5
#define MANTIS_V3_WIRE_CRC_SIZE 2

#define MANTIS_COMMAND_V3_TYPE 0x01
#define MANTIS_COMMAND_V3_PAYLOAD_LENGTH 249
#define MANTIS_COMMAND_V3_SIZE sizeof(MantisCommandV3_t)
#define MANTIS_COMMAND_V3_WIRE_SIZE \
    (MANTIS_V3_WIRE_PREFIX_SIZE + MANTIS_COMMAND_V3_PAYLOAD_LENGTH + MANTIS_V3_WIRE_CRC_SIZE)

#define MANTIS_FEEDBACK_V3_TYPE 0x02
#define MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH 248
#define MANTIS_FEEDBACK_V3_SIZE sizeof(MantisFeedbackV3_t)
#define MANTIS_FEEDBACK_V3_WIRE_SIZE \
    (MANTIS_V3_WIRE_PREFIX_SIZE + MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH + MANTIS_V3_WIRE_CRC_SIZE)

// Standalone chassis/lift V3 protocol. It uses the same 55 AA + CRC wire
// envelope as the full robot, but has its own frame types and compact payload.
#define MANTIS_COMMAND_V3_CHASSIS_TYPE 0x21
#define MANTIS_FEEDBACK_V3_CHASSIS_TYPE 0x22
#define MANTIS_V3_CHASSIS_PAYLOAD_LENGTH 37
#define MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH MANTIS_V3_CHASSIS_PAYLOAD_LENGTH
#define MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH MANTIS_V3_CHASSIS_PAYLOAD_LENGTH
#define MANTIS_COMMAND_V3_CHASSIS_WIRE_SIZE \
    (MANTIS_V3_WIRE_PREFIX_SIZE + MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH + MANTIS_V3_WIRE_CRC_SIZE)
#define MANTIS_FEEDBACK_V3_CHASSIS_WIRE_SIZE \
    (MANTIS_V3_WIRE_PREFIX_SIZE + MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH + MANTIS_V3_WIRE_CRC_SIZE)

#define MANTIS_PROTOCOL_V1 1
#define MANTIS_PROTOCOL_V2 2
#define MANTIS_PROTOCOL_V3 3

#pragma pack(1)
typedef struct
{                         // 都使用朴素机器人坐标系,前x,左y,上z
    uint8_t frame_header; // 帧头 0x1F
    uint8_t frame_length; // 帧长度 94
    uint8_t ctrl_src_flag;  // set to 0b10 to teleoperate robot with VR joystick.
    // chassis
    float vx;
    float vy;
    float omega;
    // pelvis velocity
    float pelvis_velocity;
    // left arm
    float left_shoulder_pitch_cmd;
    float left_shoulder_yaw_cmd;
    float left_shoulder_roll_cmd;
    float left_elbow_pitch_cmd;        // 修正：添加pitch后缀
    float left_wrist_roll_cmd;
    float left_wrist_pitch_cmd;
    float left_wrist_yaw_cmd;
    float left_gripper_cmd;
    // right arm
    float right_shoulder_pitch_cmd;
    float right_shoulder_yaw_cmd;
    float right_shoulder_roll_cmd;
    float right_elbow_pitch_cmd;       // 修正：添加pitch后缀
    float right_wrist_roll_cmd;
    float right_wrist_pitch_cmd;
    float right_wrist_yaw_cmd;
    float right_gripper_cmd;
    // neck
    float head_yaw_cmd;
    float head_pitch_cmd;
    uint8_t frame_tail;     // 0x1E
    uint16_t crc16;
} MantisCommand_t;
#pragma pack()

#pragma pack(1)
typedef struct
{                         // 新版协议（V2）：参考“嵌入式&工控机-控制帧.xlsx”
    uint8_t frame_header; // 固定值 0x1F
    uint8_t frame_length; // 固定值 254
    uint8_t ctrl_src_flag;  // V2上电信号：1-软件开机，0-软件关机

    float vx;
    float vy;
    float omega;
    float max_acc_x;
    float max_acc_y;
    float max_acc_omega;
    float pelvis_height;
    float pelvis_max_velocity;

    float left_joint_pos_0;
    float left_joint_pos_1;
    float left_joint_pos_2;
    float left_joint_pos_3;
    float left_joint_pos_4;
    float left_joint_pos_5;
    float left_joint_pos_6;

    uint8_t packet1_tail;   // 固定值 0x1E
    uint8_t packet2_header; // 固定值 0x2F

    float left_joint_pos_7;
    float right_joint_pos_0;
    float right_joint_pos_1;
    float right_joint_pos_2;
    float right_joint_pos_3;
    float right_joint_pos_4;
    float right_joint_pos_5;
    float right_joint_pos_6;
    float right_joint_pos_7;
    float head_yaw_pos;
    float head_pitch_pos;
    float left_joint_vel_0;
    float left_joint_vel_1;
    float left_joint_vel_2;
    float left_joint_vel_3;
    uint16_t packet2_crc16;  // 第2包预留/校验

    uint8_t packet2_tail;   // 固定值 0x2E
    uint8_t packet3_header; // 固定值 0x3F

    float left_joint_vel_4;
    float left_joint_vel_5;
    float left_joint_vel_6;
    float left_joint_vel_7;
    float right_joint_vel_0;
    float right_joint_vel_1;
    float right_joint_vel_2;
    float right_joint_vel_3;
    float right_joint_vel_4;
    float right_joint_vel_5;
    float right_joint_vel_6;
    float right_joint_vel_7;
    float head_yaw_vel;
    float head_pitch_vel;
    float left_joint_torque_0;
    uint16_t packet3_crc16;  // 第3包预留/校验

    uint8_t packet3_tail;   // 固定值 0x3E
    uint8_t packet4_header; // 固定值 0x4F

    float left_joint_torque_1;
    float left_joint_torque_2;
    float left_joint_torque_3;
    float left_joint_torque_4;
    float left_joint_torque_5;
    float left_joint_torque_6;
    float left_joint_torque_7;
    float right_joint_torque_0;
    float right_joint_torque_1;
    float right_joint_torque_2;
    float right_joint_torque_3;
    float right_joint_torque_4;
    float right_joint_torque_5;
    float right_joint_torque_6;
    float right_joint_torque_7;
    uint8_t frame_tail; // 固定值 0x4E
} MantisCommandV2_t;
#pragma pack()

static_assert(sizeof(MantisCommandV2_t) == 254, "MantisCommandV2_t size must be 254 bytes");

#pragma pack(1)
typedef struct
{                                  // V3 控制协议：帧类型 0x01，负载长度 249
    uint8_t frame_type;            // 固定值 0x01
    uint8_t payload_length;        // 固定值 249
    uint8_t control_flag;          // bit1~bit5 见协议文档

    float vx;
    float vy;
    float omega;
    float max_acc_x;
    float max_acc_y;
    float max_acc_omega;
    float pelvis_height;
    float pelvis_velocity;

    float left_joint_pos_0;
    float left_joint_pos_1;
    float left_joint_pos_2;
    float left_joint_pos_3;
    float left_joint_pos_4;
    float left_joint_pos_5;
    float left_joint_pos_6;
    float left_joint_pos_7;

    float right_joint_pos_0;
    float right_joint_pos_1;
    float right_joint_pos_2;
    float right_joint_pos_3;
    float right_joint_pos_4;
    float right_joint_pos_5;
    float right_joint_pos_6;
    float right_joint_pos_7;

    float waist_pos;
    float head_yaw_pos;
    float head_pitch_pos;

    float left_joint_max_vel_0;
    float left_joint_max_vel_1;
    float left_joint_max_vel_2;
    float left_joint_max_vel_3;
    float left_joint_max_vel_4;
    float left_joint_max_vel_5;
    float left_joint_max_vel_6;
    float left_joint_max_vel_7;

    float right_joint_max_vel_0;
    float right_joint_max_vel_1;
    float right_joint_max_vel_2;
    float right_joint_max_vel_3;
    float right_joint_max_vel_4;
    float right_joint_max_vel_5;
    float right_joint_max_vel_6;
    float right_joint_max_vel_7;

    float waist_max_vel;
    float head_yaw_max_vel; // Byte177: temporarily carries head roll position
    float head_pitch_max_vel;

    float left_joint_torque_0;
    float left_joint_torque_1;
    float left_joint_torque_2;
    float left_joint_torque_3;
    float left_joint_torque_4;
    float left_joint_torque_5;
    float left_joint_torque_6;
    float left_joint_torque_7;

    float right_joint_torque_0;
    float right_joint_torque_1;
    float right_joint_torque_2;
    float right_joint_torque_3;
    float right_joint_torque_4;
    float right_joint_torque_5;
    float right_joint_torque_6;
    float right_joint_torque_7;
} MantisCommandV3_t;
#pragma pack()

static_assert(sizeof(MantisCommandV3_t) == 251, "MantisCommandV3_t size must be 251 bytes");

#pragma pack(1)
typedef struct
{                                      // 独立底盘/滑台 V3 控制帧：帧类型 0x21，负载长度 37
    uint8_t frame_type;                // 固定值 0x21
    uint8_t payload_length;            // 固定值 37
    uint8_t control_flag;              // bit0：软件开关
    float vx;                          // m/s
    float vy;                          // m/s
    float omega;                       // rad/s
    float max_acc_x;                   // m/s^2
    float max_acc_y;                   // m/s^2
    float max_acc_omega;               // rad/s^2
    float pelvis_height;               // mm
    float pelvis_max_velocity;         // mm/s
    float pelvis_acceleration;         // mm/s^2
} MantisCommandV3Chassis_t;
#pragma pack()

static_assert(
    sizeof(MantisCommandV3Chassis_t) == 2 + MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH,
    "MantisCommandV3Chassis_t size must be 39 bytes");

#pragma pack(1)
typedef struct
{                          // 都使用朴素机器人坐标系,前x,左y,上z
    uint8_t frame_header;     // 固定值 0x2F
    uint8_t frame_length;   // 帧长度 101
    float wave_hand_switch; // 摆手启动开关：1=开，非1=关
    float res2; // 保留字节
    float res3; // 保留字节
    // pelvis height
    float pelvis_height;
    // left arm
    float left_shoulder_pitch_cur;
    float left_shoulder_yaw_cur;
    float left_shoulder_roll_cur;
    float left_elbow_pitch_cur;        // 修正：添加pitch后缀
    float left_wrist_roll_cur;
    float left_wrist_pitch_cur;
    float left_wrist_yaw_cur;
    float left_gripper_cur;
    // right arm
    float right_shoulder_pitch_cur;
    float right_shoulder_yaw_cur;
    float right_shoulder_roll_cur;
    float right_elbow_pitch_cur;       // 修正：添加pitch后缀
    float right_wrist_roll_cur;
    float right_wrist_pitch_cur;
    float right_wrist_yaw_cur;
    float right_gripper_cur;
    // neck
    float head_pitch_cur;
    float head_yaw_cur;
    // force feedback
    float left_force_feedback;
    float right_force_feedback;
    uint8_t frame_tail;     // 固定值 0x1E
    uint16_t crc16;
} MantisFeedback_t;
#pragma pack()

#pragma pack(1)
typedef struct
{                                // 新版反馈协议（V2）：总长 254 字节
    uint8_t frame_header;        // 固定值 0x1F
    uint8_t frame_length;        // 固定值 254

    float wheel_front_left_enc;
    float wheel_front_left_vel;
    float wheel_front_right_enc;
    float wheel_front_right_vel;
    float wheel_middle_enc;
    float wheel_middle_vel;
    float robot_heading;
    float robot_heading_vel;
    float pelvis_height;
    float pelvis_max_velocity;

    float left_joint_pos_0;
    float left_joint_pos_1;
    float left_joint_pos_2;
    float left_joint_pos_3;
    float left_joint_pos_4;

    uint8_t packet1_crc8;        // 第1包预留/校验

    uint8_t packet1_tail;        // 固定值 0x1E
    uint8_t packet2_header;      // 固定值 0x2F

    float left_joint_pos_5;
    float left_joint_pos_6;
    float left_joint_pos_7;
    float right_joint_pos_0;
    float right_joint_pos_1;
    float right_joint_pos_2;
    float right_joint_pos_3;
    float right_joint_pos_4;
    float right_joint_pos_5;
    float right_joint_pos_6;
    float right_joint_pos_7;
    float head_yaw_pos;
    float head_pitch_pos;
    float left_joint_vel_0;
    float left_joint_vel_1;
    uint16_t packet2_crc16;      // 第2包预留/校验

    uint8_t packet2_tail;        // 固定值 0x2E
    uint8_t packet3_header;      // 固定值 0x3F

    float left_joint_vel_2;
    float left_joint_vel_3;
    float left_joint_vel_4;
    float left_joint_vel_5;
    float left_joint_vel_6;
    float left_joint_vel_7;
    float right_joint_vel_0;
    float right_joint_vel_1;
    float right_joint_vel_2;
    float right_joint_vel_3;
    float right_joint_vel_4;
    float right_joint_vel_5;
    float right_joint_vel_6;
    float right_joint_vel_7;
    float left_joint_torque_0;
    uint16_t packet3_crc16;      // 第3包预留/校验

    uint8_t packet3_tail;        // 固定值 0x3E
    uint8_t packet4_header;      // 固定值 0x4F

    float left_joint_torque_1;
    float left_joint_torque_2;
    float left_joint_torque_3;
    float left_joint_torque_4;
    float left_joint_torque_5;
    float left_joint_torque_6;
    float left_joint_torque_7;
    float right_joint_torque_0;
    float right_joint_torque_1;
    float right_joint_torque_2;
    float right_joint_torque_3;
    float right_joint_torque_4;
    float right_joint_torque_5;
    float right_joint_torque_6;
    float right_joint_torque_7;
    uint8_t frame_tail;          // 固定值 0x1E
} MantisFeedbackV2_t;
#pragma pack()

static_assert(sizeof(MantisFeedbackV2_t) == 254, "MantisFeedbackV2_t size must be 254 bytes");

#pragma pack(1)
typedef struct
{                                  // V3 反馈协议：帧类型 0x02，负载长度 248
    uint8_t frame_type;            // 固定值 0x02
    uint8_t payload_length;        // 固定值 248
    uint8_t status_flags;          // 总状态标志
    uint8_t left_arm_status_flags; // 左臂 8 路状态
    uint8_t right_arm_status_flags;// 右臂 8 路状态
    uint8_t chassis_status_flags;  // 底盘状态

    float chassis_vx;
    float chassis_vy;
    float chassis_omega;
    float odom_x;
    float odom_y;
    float robot_heading;
    float robot_heading_vel;
    float pelvis_height;
    float pelvis_velocity;

    float left_joint_pos_0;
    float left_joint_pos_1;
    float left_joint_pos_2;
    float left_joint_pos_3;
    float left_joint_pos_4;
    float left_joint_pos_5;
    float left_joint_pos_6;
    float left_joint_pos_7;

    float right_joint_pos_0;
    float right_joint_pos_1;
    float right_joint_pos_2;
    float right_joint_pos_3;
    float right_joint_pos_4;
    float right_joint_pos_5;
    float right_joint_pos_6;
    float right_joint_pos_7;

    float waist_pos;
    float head_yaw_pos;
    float head_pitch_pos;

    float left_joint_vel_0;
    float left_joint_vel_1;
    float left_joint_vel_2;
    float left_joint_vel_3;
    float left_joint_vel_4;
    float left_joint_vel_5;
    float left_joint_vel_6;
    float left_joint_vel_7;

    float right_joint_vel_0;
    float right_joint_vel_1;
    float right_joint_vel_2;
    float right_joint_vel_3;
    float right_joint_vel_4;
    float right_joint_vel_5;
    float right_joint_vel_6;
    float right_joint_vel_7;

    float waist_vel;

    float left_joint_torque_0;
    float left_joint_torque_1;
    float left_joint_torque_2;
    float left_joint_torque_3;
    float left_joint_torque_4;
    float left_joint_torque_5;
    float left_joint_torque_6;
    float left_joint_torque_7;

    float right_joint_torque_0;
    float right_joint_torque_1;
    float right_joint_torque_2;
    float right_joint_torque_3;
    float right_joint_torque_4;
    float right_joint_torque_5;
    float right_joint_torque_6;
    float right_joint_torque_7;
} MantisFeedbackV3_t;
#pragma pack()

static_assert(sizeof(MantisFeedbackV3_t) == 250, "MantisFeedbackV3_t size must be 250 bytes");

#pragma pack(1)
typedef struct
{                                      // 独立底盘/滑台 V3 反馈帧：帧类型 0x22，负载长度 37
    uint8_t frame_type;                // 固定值 0x22
    uint8_t payload_length;            // 固定值 37
    uint8_t status_flags;              // bit0 软件开关，bit1 底盘，bit2 滑台，bit3~6 轮电机
    float chassis_vx;                  // m/s
    float chassis_vy;                  // m/s
    float chassis_omega;               // rad/s
    float odom_x;                      // m
    float odom_y;                      // m
    float robot_heading;               // rad
    float robot_heading_vel;           // rad/s
    float pelvis_height;               // mm
    float pelvis_velocity;             // mm/s
} MantisFeedbackV3Chassis_t;
#pragma pack()

static_assert(
    sizeof(MantisFeedbackV3Chassis_t) == 2 + MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH,
    "MantisFeedbackV3Chassis_t size must be 39 bytes");

#define CONTROLLER_COMMAND_FRAME_SIZE sizeof(ControllerCommand_t)
#define CONTROLLER_COMMAND_FRAME_HEADER 0x5A
#define CONTROLLER_COMMAND_FRAME_TAIL 0x05

#define FORCE_FEEDBACK_FRAME_SIZE sizeof(ForceFeedback_t)
#define FORCE_FEEDBACK_FRAME_HEADER 0xC1
#define FORCE_FEEDBACK_FRAME_TAIL 0x8F

#if 0   // Deprecated Communication Protocol
    #pragma pack(1)
    typedef struct 
    {
        uint8_t frame_header;   // 0x5A
        float right_vel_x;
        float right_vel_y;
        float right_motor_vel;
        uint8_t right_button_val;
        // left controller
        float left_vel_x;
        float left_vel_y;
        float left_motor_vel;
        uint8_t left_button_val;
        uint8_t LRC8;
        uint8_t frame_tail; // 0x05
    } ControllerCommand_t;
    #pragma pack()

    #pragma pack(1)
    typedef struct 
    {
        uint8_t frame_header;   //0xC1
        uint8_t frame_length;
        uint8_t status;
        float right_force_feedback;
        float left_force_feedback;
        uint8_t LRC8;
        uint8_t frame_tail; // 0x8F
    } ForceFeedback_t;
    #pragma pack()
#endif

#pragma pack(1)
typedef struct 
{
    uint8_t frame_header;   // 0x5A
    float position;
    uint8_t LRC8;
    uint8_t frame_tail;     // 0x05
} ControllerCommand_t;
#pragma pack()

#pragma pack(1)
typedef struct 
{
    uint8_t frame_header;   //0xC1
    uint8_t status;
    float force_feedback;
    uint8_t LRC;
    uint8_t frame_tail; // 0x8F
} ForceFeedback_t;
#pragma pack()

#pragma pack(1)
typedef struct 
{
    uint8_t frame_header; // 帧头 0x01
    float f1;
    float f2;
    float f3;
    uint16_t crc16; // CRC16 校验（暂无）
    uint8_t frame_tail; // 帧尾 0x10
} SerialTest_t;
#pragma pack()

#endif // PROTOCOL_HPP
