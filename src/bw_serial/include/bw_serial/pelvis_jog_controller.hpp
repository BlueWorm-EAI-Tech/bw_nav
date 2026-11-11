#ifndef BW_SERIAL__PELVIS_JOG_CONTROLLER_HPP_
#define BW_SERIAL__PELVIS_JOG_CONTROLLER_HPP_

#include <string>

namespace bw_serial
{

struct PelvisJogConfig
{
  float default_height_mm{1000.0F};
  float min_height_mm{500.0F};
  float max_height_mm{1500.0F};
  float max_speed_mm_s{200.0F};
  float input_deadzone{0.1F};
  bool startup_init_enabled{true};
  float startup_init_tolerance_mm{10.0F};
  double startup_init_timeout_sec{8.0};
};

struct PelvisJogOutput
{
  float normalized_input{0.0F};
  float target_height_mm{1000.0F};
  float max_velocity_mm_s{0.0F};
  bool startup_initializing{false};
  bool startup_initialization_completed{false};
};

class PelvisJogController
{
public:
  explicit PelvisJogController(const PelvisJogConfig & config = PelvisJogConfig());

  PelvisJogOutput step(float raw_input, double dt_sec);
  PelvisJogOutput startup_initialization_step(bool has_feedback, float feedback_height_mm, double dt_sec);
  void reset();
  void stop_startup_initialization();
  void sync_target_height_mm(float feedback_height_mm);
  float target_height_mm() const;
  bool startup_initialization_active() const;

private:
  static float apply_deadzone(float raw_input, float deadzone);
  static float clamp(float value, float min_value, float max_value);

  PelvisJogConfig config_;
  float target_height_mm_;
  bool startup_init_active_;
  double startup_init_elapsed_sec_;
};

// 根据机器人型号 (1.0 / 2.0 / 3.0 / standard) 返回滑台 jog 的默认起点与限位。
PelvisJogConfig default_pelvis_jog_config_for_robot_version(const std::string & robot_version);

// 兼容旧调用：v2 -> 2.0 滑台参数，v3 -> 3.0 滑台参数。
PelvisJogConfig default_pelvis_jog_config_for_protocol(const std::string & protocol_version);

}  // namespace bw_serial

#endif  // BW_SERIAL__PELVIS_JOG_CONTROLLER_HPP_
