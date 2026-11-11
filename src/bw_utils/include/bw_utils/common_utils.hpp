#ifndef BW_UTILS__COMMON_UTILS_HPP
#define BW_UTILS__COMMON_UTILS_HPP

#include <chrono>

namespace bw_utils {
    class Utils {
    public:
        Utils() {};
        ~Utils() = default;

        bool measure_frequency_interval()
        {
            auto now = std::chrono::steady_clock::now();
            if (first)
            {
                first = false;
            }
            else
            {
                auto interval_ = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
                interval = static_cast<int32_t>(interval_);
                frequency = 1000.0 / interval; // Convert to Hz
            }
            last_time = now;
            return first;
        }
        
        float get_frequency() const { return frequency; }
        int32_t get_interval() const { return interval; }
        
    private:
        bool first = true;
        std::chrono::steady_clock::time_point last_time;
        float frequency = 0.0;  // unit: [hz]
        int32_t interval = 0; // unit: [ms]
    };
}

#endif