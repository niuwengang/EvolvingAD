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
/**
 * @brief logrecord init
 * @param[in] log_folder_path
 * @param[in] log_name
 * @return frequency
 * @note
 * spdlog level
 * trace = SPDLOG_LEVEL_TRACE, // low
 * debug = SPDLOG_LEVEL_DEBUG,
 * info = SPDLOG_LEVEL_INFO,
 * warn = SPDLOG_LEVEL_WARN,
 * err = SPDLOG_LEVEL_ERROR,
 * critical = SPDLOG_LEVEL_CRITICAL, // high
 * off = SPDLOG_LEVEL_OFF,
 */
LogRecord::LogRecord(const std::string log_folder_path, const std::string log_name)
{
    /*[1]--create log file*/
    std::string log_file_path = log_folder_path + "/" + log_name + ".txt";
    std::ofstream file_writer(log_file_path, std::ios_base::out); // over write not append

    file_ = spdlog::basic_logger_mt(log_name + ".", log_file_path);
    terminal_ = spdlog::stdout_color_mt(log_name);

    file_->set_level(spdlog::level::trace);
    file_->flush_on(spdlog::level::trace);

    terminal_->set_level(spdlog::level::trace);
    terminal_->flush_on(spdlog::level::trace);
}
} // namespace evolving_ad_ns