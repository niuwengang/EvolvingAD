#include "monitor.hpp"

namespace Tools
{
void TimeRecord::Start()
{
    start_record_ = std::chrono::steady_clock::now();
}
double TimeRecord::End()
{
    end_record_ = std::chrono::steady_clock::now();

    double duration_millsecond = std::chrono::duration<double, std::milli>(end_record_ - start_record_).count();
    // std::cout << std::fixed << "程序耗时:" << duration_millsecond << "毫秒 " << 1000.0 / duration_millsecond << " HZ"
    //           << std::endl;
    return 1000.0 / duration_millsecond;
}

LogRecord::LogRecord(const std::string log_folder_path, const std::string log_name)
{
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
