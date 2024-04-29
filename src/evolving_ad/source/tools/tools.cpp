/**
 * @file    tools.hpp
 * @brief   tools
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 * spdlog level
 * trace = SPDLOG_LEVEL_TRACE, // low
 * debug = SPDLOG_LEVEL_DEBUG,
 * info = SPDLOG_LEVEL_INFO,
 * warn = SPDLOG_LEVEL_WARN,
 * err = SPDLOG_LEVEL_ERROR,
 * critical = SPDLOG_LEVEL_CRITICAL, // high
 * off = SPDLOG_LEVEL_OFF,
 */

#include "tools.hpp"

namespace evolving_ad_ns
{

/**
 * @brief record time start
 * @param[in] none
 * @return
 */
void TimeRecord::Start()
{
    start_record_ = std::chrono::steady_clock::now();
}

/**
 * @brief record time end
 * @param[in] none
 * @return frequency
 */
double TimeRecord::GetFrequency(const unsigned int slide_windows)
{
    end_record_ = std::chrono::steady_clock::now();

    double duration = std::chrono::duration<double, std::milli>(end_record_ - start_record_).count(); // ms

    time_queue_.push_back(duration);
    sum_time_ += duration;
    if (time_queue_.size() > slide_windows)
    {
        sum_time_ -= time_queue_.front();
        time_queue_.pop_front();
    }

    return static_cast<double>(time_queue_.size()) * 1000.0 / sum_time_;
}
}