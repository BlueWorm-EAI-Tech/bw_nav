#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "protocol.hpp"
#include "bw_serial/v3_frame_codec.hpp"

namespace bw_serial
{
namespace
{

TEST(V3FrameCodecTest, EncodeCommandFrameUsesDocumentWireFormat)
{
    MantisCommandV3_t command{};
    command.frame_type = MANTIS_COMMAND_V3_TYPE;
    command.payload_length = MANTIS_COMMAND_V3_PAYLOAD_LENGTH;
    command.control_flag = 0x1F;
    command.vx = 0.25F;
    command.vy = -0.5F;
    command.pelvis_height = 1000.0F;

    const auto frame = encode_v3_command_frame(command);

    ASSERT_EQ(frame.size(), MANTIS_COMMAND_V3_WIRE_SIZE);
    EXPECT_EQ(frame[0], MANTIS_V3_FRAME_HEADER_BYTE_0);
    EXPECT_EQ(frame[1], MANTIS_V3_FRAME_HEADER_BYTE_1);
    EXPECT_EQ(frame[2], MANTIS_COMMAND_V3_TYPE);
    EXPECT_EQ(frame[3], static_cast<uint8_t>(MANTIS_COMMAND_V3_PAYLOAD_LENGTH & 0xFF));
    EXPECT_EQ(frame[4], static_cast<uint8_t>((MANTIS_COMMAND_V3_PAYLOAD_LENGTH >> 8) & 0xFF));
    EXPECT_EQ(frame[5], 0x1F);

    const uint16_t encoded_crc = static_cast<uint16_t>(frame[frame.size() - 2]) |
                                 (static_cast<uint16_t>(frame[frame.size() - 1]) << 8);
    const uint16_t expected_crc = compute_v3_frame_crc(
        MANTIS_COMMAND_V3_TYPE,
        MANTIS_COMMAND_V3_PAYLOAD_LENGTH,
        frame.data() + MANTIS_V3_WIRE_PREFIX_SIZE,
        MANTIS_COMMAND_V3_PAYLOAD_LENGTH);
    EXPECT_EQ(encoded_crc, expected_crc);
}

TEST(V3FrameCodecTest, EncodeChassisCommandFrameUsesCompactPayload)
{
    MantisCommandV3Chassis_t command{};
    command.frame_type = MANTIS_COMMAND_V3_CHASSIS_TYPE;
    command.payload_length = MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH;
    command.control_flag = 0x01;
    command.vx = 0.3F;
    command.pelvis_height = 1200.0F;
    command.pelvis_max_velocity = 250.0F;
    command.pelvis_acceleration = 1000.0F;

    const auto frame = encode_v3_chassis_command_frame(command);

    ASSERT_EQ(frame.size(), MANTIS_COMMAND_V3_CHASSIS_WIRE_SIZE);
    EXPECT_EQ(frame[0], MANTIS_V3_FRAME_HEADER_BYTE_0);
    EXPECT_EQ(frame[1], MANTIS_V3_FRAME_HEADER_BYTE_1);
    EXPECT_EQ(frame[2], MANTIS_COMMAND_V3_CHASSIS_TYPE);
    EXPECT_EQ(frame[3], MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH);
    EXPECT_EQ(frame[4], 0U);
    EXPECT_EQ(frame[5], command.control_flag);

    const uint16_t encoded_crc = static_cast<uint16_t>(frame[frame.size() - 2]) |
                                 (static_cast<uint16_t>(frame[frame.size() - 1]) << 8);
    EXPECT_EQ(
        encoded_crc,
        compute_v3_frame_crc(
            MANTIS_COMMAND_V3_CHASSIS_TYPE,
            MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH,
            frame.data() + MANTIS_V3_WIRE_PREFIX_SIZE,
            MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH));
}

TEST(V3FrameCodecTest, DecodeChassisFeedbackFrameAcceptsValidWireFrame)
{
    MantisFeedbackV3Chassis_t expected{};
    expected.frame_type = MANTIS_FEEDBACK_V3_CHASSIS_TYPE;
    expected.payload_length = MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH;
    expected.status_flags = 0x07;
    expected.chassis_vx = 0.4F;
    expected.odom_x = 1.25F;
    expected.robot_heading = -0.2F;
    expected.pelvis_height = 1350.0F;
    expected.pelvis_velocity = -50.0F;

    const auto frame = encode_v3_frame(
        MANTIS_FEEDBACK_V3_CHASSIS_TYPE,
        MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH,
        reinterpret_cast<const uint8_t *>(&expected.status_flags));

    MantisFeedbackV3Chassis_t decoded{};
    ASSERT_TRUE(decode_v3_chassis_feedback_frame(frame, &decoded));
    EXPECT_EQ(decoded.status_flags, expected.status_flags);
    EXPECT_FLOAT_EQ(decoded.chassis_vx, expected.chassis_vx);
    EXPECT_FLOAT_EQ(decoded.odom_x, expected.odom_x);
    EXPECT_FLOAT_EQ(decoded.robot_heading, expected.robot_heading);
    EXPECT_FLOAT_EQ(decoded.pelvis_height, expected.pelvis_height);
    EXPECT_FLOAT_EQ(decoded.pelvis_velocity, expected.pelvis_velocity);
}

TEST(V3FrameCodecTest, DecodeChassisFeedbackFrameRejectsCrcMismatch)
{
    std::array<uint8_t, MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH> payload{};
    const auto valid_frame = encode_v3_frame(
        MANTIS_FEEDBACK_V3_CHASSIS_TYPE,
        MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH,
        payload.data());

    auto corrupted_frame = valid_frame;
    corrupted_frame[6] ^= 0x01;

    MantisFeedbackV3Chassis_t decoded{};
    EXPECT_FALSE(decode_v3_chassis_feedback_frame(corrupted_frame, &decoded));
}

TEST(V3FrameCodecTest, ChassisPayloadStartsAtExpectedOffsets)
{
    EXPECT_EQ(offsetof(MantisCommandV3Chassis_t, control_flag), 2U);
    EXPECT_EQ(offsetof(MantisCommandV3Chassis_t, vx), 3U);
    EXPECT_EQ(offsetof(MantisCommandV3Chassis_t, pelvis_height), 27U);
    EXPECT_EQ(offsetof(MantisFeedbackV3Chassis_t, status_flags), 2U);
    EXPECT_EQ(offsetof(MantisFeedbackV3Chassis_t, chassis_vx), 3U);
    EXPECT_EQ(offsetof(MantisFeedbackV3Chassis_t, pelvis_height), 31U);
}

TEST(V3FrameCodecTest, HeadFieldsMatchDocumentPayloadOffsets)
{
    EXPECT_EQ(offsetof(MantisCommandV3_t, head_yaw_pos) - 2U, 101U);
    EXPECT_EQ(offsetof(MantisCommandV3_t, head_pitch_pos) - 2U, 105U);
    EXPECT_EQ(offsetof(MantisCommandV3_t, head_yaw_max_vel) - 2U, 177U);
}

TEST(V3FrameCodecTest, ComputeFrameCrcMatchesDocumentModbusExample)
{
    const uint8_t payload[] = {0x11, 0x22, 0x33};
    const uint16_t crc = compute_v3_frame_crc(0x01, 3, payload, 3);
    EXPECT_EQ(crc, 0x7A4D);
}

TEST(V3FrameCodecTest, DecodeFeedbackFrameAcceptsValidWireFrame)
{
    MantisFeedbackV3_t expected{};
    expected.frame_type = MANTIS_FEEDBACK_V3_TYPE;
    expected.payload_length = MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH;
    expected.status_flags = 0x12;
    expected.left_arm_status_flags = 0x34;
    expected.right_arm_status_flags = 0x56;
    expected.chassis_status_flags = 0x78;
    expected.pelvis_height = 888.0F;
    expected.left_joint_pos_0 = 0.1F;
    expected.right_joint_pos_7 = 0.9F;
    expected.waist_pos = -12.0F;

    const auto frame = encode_v3_frame(
        MANTIS_FEEDBACK_V3_TYPE,
        MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH,
        reinterpret_cast<const uint8_t *>(&expected.status_flags));

    MantisFeedbackV3_t decoded{};
    ASSERT_TRUE(decode_v3_feedback_frame(frame, &decoded));
    EXPECT_EQ(decoded.frame_type, expected.frame_type);
    EXPECT_EQ(decoded.payload_length, expected.payload_length);
    EXPECT_EQ(decoded.status_flags, expected.status_flags);
    EXPECT_EQ(decoded.left_arm_status_flags, expected.left_arm_status_flags);
    EXPECT_EQ(decoded.right_arm_status_flags, expected.right_arm_status_flags);
    EXPECT_EQ(decoded.chassis_status_flags, expected.chassis_status_flags);
    EXPECT_FLOAT_EQ(decoded.pelvis_height, expected.pelvis_height);
    EXPECT_FLOAT_EQ(decoded.left_joint_pos_0, expected.left_joint_pos_0);
    EXPECT_FLOAT_EQ(decoded.right_joint_pos_7, expected.right_joint_pos_7);
    EXPECT_FLOAT_EQ(decoded.waist_pos, expected.waist_pos);
}

TEST(V3FrameCodecTest, DecodeFeedbackFrameRejectsCrcMismatch)
{
    std::array<uint8_t, MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH> payload{};
    const auto valid_frame = encode_v3_frame(
        MANTIS_FEEDBACK_V3_TYPE,
        MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH,
        payload.data());

    auto corrupted_frame = valid_frame;
    corrupted_frame.back() ^= 0xFF;

    MantisFeedbackV3_t decoded{};
    EXPECT_FALSE(decode_v3_feedback_frame(corrupted_frame, &decoded));
}

}  // namespace
}  // namespace bw_serial
