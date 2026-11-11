#ifndef BW_SERIAL__CHASSIS_COMMAND_WATCHDOG_HPP_
#define BW_SERIAL__CHASSIS_COMMAND_WATCHDOG_HPP_

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace bw_serial
{

class ChassisCommandWatchdog
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit ChassisCommandWatchdog(double timeout_sec = 0.5)
  {
    set_timeout(timeout_sec);
  }

  void set_timeout(double timeout_sec)
  {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
      throw std::invalid_argument("chassis command timeout must be positive");
    }
    timeout_ = std::chrono::duration<double>(timeout_sec);
  }

  void observe(TimePoint now = Clock::now())
  {
    last_command_time_ = now;
    has_command_ = true;
  }

  bool expired(TimePoint now = Clock::now()) const
  {
    return has_command_ && now - last_command_time_ > timeout_;
  }

private:
  std::chrono::duration<double> timeout_{0.5};
  TimePoint last_command_time_{};
  bool has_command_{false};
};

} // namespace bw_serial

#endif // BW_SERIAL__CHASSIS_COMMAND_WATCHDOG_HPP_
