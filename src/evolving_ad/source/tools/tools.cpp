/**
 * @file    tools.hpp
 * @brief   tools
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
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

/**
 * @brief TrajRecord init
 * @param[in] traj_folder_path
 * @param[in] file_name
 * @return frequency
 * @note
 */
TrajRecord::TrajRecord(const std::string &traj_folder_path, const std::string &file_name)
{
    FileManager::CreateFolder(traj_folder_path);
    FileManager::CreateTxtFile(traj_ofs_, traj_folder_path + "/" + file_name + ".txt");
}

/**
 * @brief TrajRecord init
 * @param[in] pose
 * @param[in] time_stamp
 * @return frequency
 * @note use tum format to save pose
 */
void TrajRecord::SavePose(const Eigen::Matrix4f &pose, const double time_stamp)
{
    std::vector<double> tum_output(8);

    tum_output[0] = time_stamp; // t

    tum_output[1] = pose(0, 3); // x
    tum_output[2] = pose(1, 3); // y
    tum_output[3] = pose(2, 3); // z

    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    tum_output[4] = q.x(); // qx
    tum_output[5] = q.y(); // qy
    tum_output[6] = q.z(); // qz
    tum_output[7] = q.w(); // qw

    for (int i = 0; i < 8; i++)
    {
        traj_ofs_ << std::fixed << tum_output[i];

        if (i == 7)
        {
            traj_ofs_ << std::endl;
            return;
        }
        traj_ofs_ << " ";
    }
    traj_ofs_ << std::endl;
}

/**
 * @brief create folder
 * @param[in] folder_path
 * @return bool
 * @note use tum format to save pose
 */
bool FileManager::CreateFolder(const std::string &folder_path)
{
    std::filesystem::path fs_folder_path(folder_path);

    if (!std::filesystem::exists(fs_folder_path))
    {
        try
        {
            std::filesystem::create_directories(fs_folder_path);
            return true;
        }
        catch (const std::exception &e)
        {
            return false;
        }
    }
    else
    {
        return true;
    }
}

/**
 * @brief create txt file
 * @param[in] ofs
 * @param[in] file_path
 * @return bool
 * @note
 */
bool FileManager::CreateTxtFile(std::ofstream &ofs, const std::string file_path)
{
    // check if file exist
    if (std::filesystem::exists(file_path))
    {
        return true;
    }

    ofs.open(file_path.c_str(), std::ios::app);
    if (ofs.is_open())
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief clear folder
 * @param[in] ofs
 * @param[in] file_path
 * @return bool
 * @note use tum format to save pose
 */
bool FileManager::ClearFolder(const std::string &folder_path)
{
    std::filesystem::path fs_folder_path(folder_path);

    if (std::filesystem::exists(fs_folder_path))
    {
        std::filesystem::remove_all(fs_folder_path);
    }
    return true;
}
} // namespace evolving_ad_ns