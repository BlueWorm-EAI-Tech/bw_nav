#include "bw_serial/v3_frame_codec.hpp"

#include <cstddef>
#include <cstring>
#include <vector>

namespace bw_serial
{
namespace
{

constexpr uint16_t kModbusCrcInit = 0xFFFF;
constexpr uint16_t kModbusCrcPoly = 0xA001;

uint16_t modbus_crc16(const uint8_t *data, const uint16_t length)
{
    uint16_t crc = kModbusCrcInit;
    for (uint16_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = static_cast<uint16_t>((crc >> 1U) ^ kModbusCrcPoly);
            }
            else
            {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

}  // namespace

uint16_t compute_v3_frame_crc(
    const uint8_t frame_type,
    const uint16_t payload_length,
    const uint8_t *payload,
    const uint16_t payload_size)
{
    if (payload_length != payload_size)
    {
        return 0;
    }

    std::vector<uint8_t> crc_input;
    crc_input.reserve(3U + payload_size);
    crc_input.push_back(frame_type);
    crc_input.push_back(static_cast<uint8_t>(payload_length & 0x00FFU));
    crc_input.push_back(static_cast<uint8_t>((payload_length >> 8U) & 0x00FFU));
    if (payload_size > 0U && payload != nullptr)
    {
        crc_input.insert(crc_input.end(), payload, payload + payload_size);
    }
    return modbus_crc16(crc_input.data(), static_cast<uint16_t>(crc_input.size()));
}

std::vector<uint8_t> encode_v3_frame(
    const uint8_t frame_type,
    const uint16_t payload_length,
    const uint8_t *payload)
{
    std::vector<uint8_t> frame;
    frame.reserve(MANTIS_V3_WIRE_PREFIX_SIZE + payload_length + MANTIS_V3_WIRE_CRC_SIZE);
    frame.push_back(MANTIS_V3_FRAME_HEADER_BYTE_0);
    frame.push_back(MANTIS_V3_FRAME_HEADER_BYTE_1);
    frame.push_back(frame_type);
    frame.push_back(static_cast<uint8_t>(payload_length & 0x00FFU));
    frame.push_back(static_cast<uint8_t>((payload_length >> 8U) & 0x00FFU));
    if (payload_length > 0U && payload != nullptr)
    {
        frame.insert(frame.end(), payload, payload + payload_length);
    }

    const uint16_t crc = compute_v3_frame_crc(frame_type, payload_length, payload, payload_length);
    frame.push_back(static_cast<uint8_t>(crc & 0x00FFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8U) & 0x00FFU));
    return frame;
}

std::vector<uint8_t> encode_v3_command_frame(const MantisCommandV3_t &command)
{
    static_assert(
        offsetof(MantisCommandV3_t, control_flag) == 2,
        "V3 command payload must start at control_flag");
    static_assert(
        sizeof(MantisCommandV3_t) - offsetof(MantisCommandV3_t, control_flag) ==
            MANTIS_COMMAND_V3_PAYLOAD_LENGTH,
        "V3 command payload size must match protocol document");

    return encode_v3_frame(
        MANTIS_COMMAND_V3_TYPE,
        MANTIS_COMMAND_V3_PAYLOAD_LENGTH,
        reinterpret_cast<const uint8_t *>(&command.control_flag));
}

std::vector<uint8_t> encode_v3_chassis_command_frame(
    const MantisCommandV3Chassis_t &command)
{
    static_assert(
        offsetof(MantisCommandV3Chassis_t, control_flag) == 2,
        "V3 chassis command payload must start at control_flag");
    static_assert(
        sizeof(MantisCommandV3Chassis_t) - offsetof(MantisCommandV3Chassis_t, control_flag) ==
            MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH,
        "V3 chassis command payload size must match protocol document");

    return encode_v3_frame(
        MANTIS_COMMAND_V3_CHASSIS_TYPE,
        MANTIS_COMMAND_V3_CHASSIS_PAYLOAD_LENGTH,
        reinterpret_cast<const uint8_t *>(&command.control_flag));
}

bool decode_v3_feedback_frame(
    const std::vector<uint8_t> &frame,
    MantisFeedbackV3_t *out_feedback)
{
    static_assert(
        offsetof(MantisFeedbackV3_t, status_flags) == 2,
        "V3 feedback payload must start at status_flags");
    static_assert(
        sizeof(MantisFeedbackV3_t) - offsetof(MantisFeedbackV3_t, status_flags) ==
            MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH,
        "V3 feedback payload size must match protocol document");

    if (out_feedback == nullptr || frame.size() != MANTIS_FEEDBACK_V3_WIRE_SIZE)
    {
        return false;
    }

    if (frame[0] != MANTIS_V3_FRAME_HEADER_BYTE_0 || frame[1] != MANTIS_V3_FRAME_HEADER_BYTE_1)
    {
        return false;
    }

    const uint8_t frame_type = frame[2];
    const uint16_t payload_length =
        static_cast<uint16_t>(frame[3]) |
        (static_cast<uint16_t>(frame[4]) << 8U);
    if (frame_type != MANTIS_FEEDBACK_V3_TYPE || payload_length != MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH)
    {
        return false;
    }

    const uint16_t received_crc =
        static_cast<uint16_t>(frame[frame.size() - 2]) |
        (static_cast<uint16_t>(frame[frame.size() - 1]) << 8U);
    const uint16_t calculated_crc = compute_v3_frame_crc(
        frame_type,
        payload_length,
        frame.data() + MANTIS_V3_WIRE_PREFIX_SIZE,
        payload_length);
    if (received_crc != calculated_crc)
    {
        return false;
    }

    std::memset(out_feedback, 0, sizeof(MantisFeedbackV3_t));
    out_feedback->frame_type = frame_type;
    out_feedback->payload_length = static_cast<uint8_t>(payload_length);
    std::memcpy(
        &out_feedback->status_flags,
        frame.data() + MANTIS_V3_WIRE_PREFIX_SIZE,
        payload_length);
    return true;
}

bool decode_v3_chassis_feedback_frame(
    const std::vector<uint8_t> &frame,
    MantisFeedbackV3Chassis_t *out_feedback)
{
    static_assert(
        offsetof(MantisFeedbackV3Chassis_t, status_flags) == 2,
        "V3 chassis feedback payload must start at status_flags");
    static_assert(
        sizeof(MantisFeedbackV3Chassis_t) - offsetof(MantisFeedbackV3Chassis_t, status_flags) ==
            MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH,
        "V3 chassis feedback payload size must match protocol document");

    if (out_feedback == nullptr || frame.size() != MANTIS_FEEDBACK_V3_CHASSIS_WIRE_SIZE)
    {
        return false;
    }

    if (frame[0] != MANTIS_V3_FRAME_HEADER_BYTE_0 || frame[1] != MANTIS_V3_FRAME_HEADER_BYTE_1 ||
        frame[2] != MANTIS_FEEDBACK_V3_CHASSIS_TYPE)
    {
        return false;
    }

    const uint16_t payload_length =
        static_cast<uint16_t>(frame[3]) |
        (static_cast<uint16_t>(frame[4]) << 8U);
    if (payload_length != MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH)
    {
        return false;
    }

    const uint16_t received_crc =
        static_cast<uint16_t>(frame[frame.size() - 2]) |
        (static_cast<uint16_t>(frame[frame.size() - 1]) << 8U);
    const uint16_t calculated_crc = compute_v3_frame_crc(
        MANTIS_FEEDBACK_V3_CHASSIS_TYPE,
        payload_length,
        frame.data() + MANTIS_V3_WIRE_PREFIX_SIZE,
        payload_length);
    if (received_crc != calculated_crc)
    {
        return false;
    }

    std::memset(out_feedback, 0, sizeof(MantisFeedbackV3Chassis_t));
    out_feedback->frame_type = frame[2];
    out_feedback->payload_length = static_cast<uint8_t>(payload_length);
    std::memcpy(
        &out_feedback->status_flags,
        frame.data() + MANTIS_V3_WIRE_PREFIX_SIZE,
        payload_length);
    return true;
}

}  // namespace bw_serial
