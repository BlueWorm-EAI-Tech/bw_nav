#ifndef BW_SERIAL__WAIST_JOG_CONTROLLER_HPP_
#define BW_SERIAL__WAIST_JOG_CONTROLLER_HPP_

namespace bw_serial
{

struct WaistJogConfig
{
  float default_position_deg{0.0F};
  float min_position_deg{-90.0F};
  float max_position_deg{5.0F};
  float max_speed_deg_s{20.0F};
  float input_deadzone{0.1F};
};

struct WaistJogOutput
{
  float normalized_input{0.0F};
  float target_position_deg{0.0F};
  float max_velocity_deg_s{0.0F};
};

/**
 * @brief V3 上半身前后弯腰 jog 控制器。
 *
 * 上层输入保持 `[-1, 1]` 归一化语义：
 * - `+1`：朝后仰方向增加角度
 * - `-1`：朝前倾方向减小角度
 * - `0`：停止
 *
 * 控制器内部维护一个目标角度估计，并输出
 * `waist_pos + waist_max_vel`，使按钮长按的体感接近速度控制。
 */
class WaistJogController
{
public:
  explicit WaistJogController(const WaistJogConfig & config = WaistJogConfig());

  WaistJogOutput step(float raw_input, double dt_sec);
  void reset();
  float target_position_deg() const;

private:
  static float apply_deadzone(float raw_input, float deadzone);
  static float clamp(float value, float min_value, float max_value);

  WaistJogConfig config_;
  float target_position_deg_;
};

}  // namespace bw_serial

#endif  // BW_SERIAL__WAIST_JOG_CONTROLLER_HPP_
