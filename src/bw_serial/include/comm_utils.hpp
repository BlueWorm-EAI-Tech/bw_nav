#ifndef BW_SERIAL__COMM_UTILS_HPP
#define BW_SERIAL__COMM_UTILS_HPP

#include <cstdint>
#include <map>
#include <string>

// 数据序列化
template <class type>
char *serialize(type &data)
{
    std::size_t size = sizeof(data);
    char *data_char = new char[size];
    memcpy(data_char, &data, size);
    return data_char;
}

// 数据反序列化
template <class type>
type deserialize(const uint8_t *data_str)
{
    type readData;
    memcpy(&readData, data_str, sizeof(readData));
    return readData;
}

inline std::map<std::string, std::pair<float, float>> joint_transform = {
    {"right_shoulder_pitch_joint", {1., 0.0}},
    {"right_shoulder_yaw_joint", {1., 0.}},
    {"right_shoulder_roll_joint", {1., 0.0}},
    {"right_elbow_pitch_joint", {1., 0.}},     // 修正：添加pitch后缀
    {"right_wrist_roll_joint", {1., 0}},
    {"right_wrist_pitch_joint", {1., 0.0}},
    {"right_wrist_yaw_joint", {1., 0.}},

    {"left_shoulder_pitch_joint", {1., 0.0}},
    {"left_shoulder_yaw_joint", {1., 0.}},
    {"left_shoulder_roll_joint", {1., 0}},
    {"left_elbow_pitch_joint", {1., 0.}},      // 修正：添加pitch后缀
    {"left_wrist_roll_joint", {1., 0}},
    {"left_wrist_pitch_joint", {1., 0.}},
    {"left_wrist_yaw_joint", {1., 0.}},
    {"pelvis_velocity_joint", {1., 0.}}
    ,
    // CamelCase aliases for formal URDF / new IK output
    {"R_Shoulder_Pitch_Joint", {-1., 0.0}},
    {"R_Shoulder_Yaw_Joint", {-1., 0.}},
    {"R_Shoulder_Roll_Joint", {1., 0.0}},
    {"R_Elbow_Pitch_Joint", {1., 0.}},
    {"R_Wrist_Roll_Joint", {-1., 0}},
    {"R_Wrist_Pitch_Joint", {-1., 0.0}},
    {"R_Wrist_Yaw_Joint", {-1., 0.}},

    {"L_Shoulder_Pitch_Joint", {-1., 0.0}},
    {"L_Shoulder_Yaw_Joint", {1., 0.}},
    {"L_Shoulder_Roll_Joint", {-1., 0}},
    {"L_Elbow_Pitch_Joint", {1., 0.}},
    {"L_Wrist_Roll_Joint", {1., 0}},
    {"L_Wrist_Pitch_Joint", {-1., 0.}},
    {"L_Wrist_Yaw_Joint", {1., 0.}}
    ,
    // standard URDF coordinates map directly to v3 joint_pos_0..6 fields.
    {"A_left_Degree1_joint", {1., 0.}},
    {"A_left_Degree2_joint", {1., 0.}},
    {"A_left_Degree3_joint", {1., 0.}},
    {"A_left_Degree4_joint", {1., 0.}},
    {"A_left_Degree5_joint", {1., 0.}},
    {"A_left_Degree6_joint", {1., 0.}},
    {"A_left_Degree7_joint", {1., 0.}},
    {"A_right_Degree1_joint", {1., 0.}},
    {"A_right_Degree2_joint", {1., 0.}},
    {"A_right_Degree3_joint", {1., 0.}},
    {"A_right_Degree4_joint", {1., 0.}},
    {"A_right_Degree5_joint", {1., 0.}},
    {"A_right_Degree6_joint", {1., 0.}},
    {"A_right_Degree7_joint", {1., 0.}}
};

inline std::map<std::string, std::pair<float, float>> head_transform = {
    {"head_pitch_joint", {-1., 0.}},
    {"head_yaw_joint", {-1., 0.}},
    {"head_roll_joint", {-1., 0.}},
    {"Head_Joint", {-1., 0.}},
    {"Neck_Joint", {-1., 0.}},
};

inline void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}

#endif
