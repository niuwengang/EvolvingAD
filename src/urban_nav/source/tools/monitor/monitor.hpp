/**
 * @file    monitor.hpp
 * @brief   系统监测: 计时、日志
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 * 1. LOG日志等级
 * trace = SPDLOG_LEVEL_TRACE, // 最低
 * debug = SPDLOG_LEVEL_DEBUG,
 * info = SPDLOG_LEVEL_INFO,
 * warn = SPDLOG_LEVEL_WARN,
 * err = SPDLOG_LEVEL_ERROR,
 * critical = SPDLOG_LEVEL_CRITICAL, // 最高
 * off = SPDLOG_LEVEL_OFF,
 */

#ifndef MONITOR_HPP
#define MONITOR_HPP

// c++ lib
#include <chrono>
#include <fstream>
#include <iostream>
// spdlog lib
#include <spdlog/sinks/basic_file_sink.h>    //循环日志
#include <spdlog/sinks/stdout_color_sinks.h> //终端输出
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
    std::chrono::steady_clock::time_point start_record_; // 开始时间
    std::chrono::steady_clock::time_point end_record_;   // 结束时间
};

class LogRecord
{
  public:
    LogRecord() = default;
    LogRecord(const std::string log_folder_path, const std::string log_name);
    ~LogRecord() = default;

    std::shared_ptr<spdlog::logger> file_ = nullptr;
    std::shared_ptr<spdlog::logger> terminal_ = nullptr;
};

} // namespace Tools

#endif