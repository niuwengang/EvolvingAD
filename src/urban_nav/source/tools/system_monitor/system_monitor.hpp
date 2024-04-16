/**
 * @file    system_monitor.hpp
 * @brief   system monitor
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

#ifndef _SYSTEM_MONITOR_HPP_
#define _SYSTEM_MONITOR_HPP_

// c++
#include <chrono>
#include <fstream>
#include <iostream>
// spdlog
#include <spdlog/sinks/basic_file_sink.h>    //log file
#include <spdlog/sinks/stdout_color_sinks.h> //terminal
#include <spdlog/spdlog.h>

namespace Tools
{
class TimeRecord
{
  public:
    TimeRecord() = default;
    ~TimeRecord() = default;
    void Start();
    double End();

  private:
    std::chrono::steady_clock::time_point start_record_; // record start
    std::chrono::steady_clock::time_point end_record_;   // record end
};

class LogRecord
{
  public:
    LogRecord() = delete;
    LogRecord(const std::string log_folder_path, const std::string log_name);
    ~LogRecord() = default;

    std::shared_ptr<spdlog::logger> file_ = nullptr;
    std::shared_ptr<spdlog::logger> terminal_ = nullptr;
};

} // namespace Tools

#endif