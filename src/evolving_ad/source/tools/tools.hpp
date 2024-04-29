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

#ifndef _TOOLS_HPP_
#define _TOOLS_HPP_

// ros
#include <ros/ros.h>
// c++
#include <chrono>
#include <deque>

namespace evolving_ad_ns
{

class TimeRecord
{
  public:
    TimeRecord() = default;
    ~TimeRecord() = default;
    void Start();
    double GetFrequency(const unsigned int slide_windows);

  private:
    std::chrono::steady_clock::time_point start_record_; // record start
    std::chrono::steady_clock::time_point end_record_;   // record end
    std::deque<double> time_queue_;
    double sum_time_ = 0.0;
};
} // namespace evolving_ad_ns

#endif //_TOOLS_HPP_

// class WatchDog
// {
//   public:
//     WatchDog() = delete;
//     WatchDog(ros::NodeHandle &nh, const double time_period = 1.0, const double time_max_delay = 10.0);
//     ~WatchDog() = default;
//     void FeedDog();
//     bool GetTimeOutStatus();

//   private:
//     void TimerCallback(const ros::TimerEvent &event);
//     ros::Timer timer_;

//     double time_accumulate_ = 0;
//     double time_period_ = 0;
//     double time_max_delay_ = 0;

//     bool time_out_flag_ = false;
// };