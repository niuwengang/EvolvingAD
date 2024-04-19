#include "system_monitor.hpp"

namespace Tools
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
double TimeRecord::End(const unsigned int slide_windows=10e2)
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

    return 1000.0 * static_cast<double>(time_queue_.size()) / sum_time_; // hz
}

/**
 * @brief logrecord init
 * @param[in] log_folder_path
 * @param[in] log_name
 * @return frequency
 * @note
 * 1. std::ios_base::out
 * open the file and write it
 * if file not exist, will create a new one
 * if file exist , will clear the content
 */
LogRecord::LogRecord(const std::string log_folder_path, const std::string log_name)
{
    /*[1]--create log file*/
    std::string log_file_path = log_folder_path + "/" + log_name + ".txt";
    std::ofstream file_writer(log_file_path, std::ios_base::out);

    file_ = spdlog::basic_logger_mt(log_name + "@", log_file_path);
    terminal_ = spdlog::stdout_color_mt(log_name);

    file_->set_level(spdlog::level::trace);
    file_->flush_on(spdlog::level::trace);

    terminal_->set_level(spdlog::level::trace);
    terminal_->flush_on(spdlog::level::trace);
}
} // namespace Tools
