#ifndef BW_SERIAL__V3_WAIST_SAFETY_HPP_
#define BW_SERIAL__V3_WAIST_SAFETY_HPP_

#include "protocol.hpp"

namespace bw_serial
{

/**
 * @brief V3 协议腰部安全默认值配置。
 *
 * 字段语义严格对应下位机 V3 控制帧：
 * - waist_pos_deg: 腰部目标角度，单位 deg
 * - waist_max_velocity_deg_s: 腰部最大角速度，单位 deg/s
 */
struct V3WaistSafetyDefaults
{
  float waist_pos_deg{0.0F};
  float waist_max_velocity_deg_s{0.0F};
};

/**
 * @brief 将 V3 协议中的腰部字段强制写入安全默认值。
 *
 * @param command 待发送的 V3 控制帧
 * @param defaults 目标安全默认值
 */
void apply_v3_waist_safety_defaults(
  MantisCommandV3_t & command,
  const V3WaistSafetyDefaults & defaults);

}  // namespace bw_serial

#endif  // BW_SERIAL__V3_WAIST_SAFETY_HPP_
