/**
 * @file    tools.hpp
 * @brief   tools
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 */

#ifndef _TOOLS_HPP_
#define _TOOLS_HPP_

#include <filesystem>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// ros
#include <ros/ros.h>
// c++
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
// spdlog
#include <spdlog/sinks/basic_file_sink.h>    //log file
#include <spdlog/sinks/stdout_color_sinks.h> //terminal
#include <spdlog/spdlog.h>

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

class LogRecord
{
  public:
    LogRecord() = delete;
    LogRecord(const std::string log_folder_path, const std::string log_name);
    ~LogRecord() = default;

    std::shared_ptr<spdlog::logger> file_ = nullptr;
    std::shared_ptr<spdlog::logger> terminal_ = nullptr;
};

class TrajRecord
{
  public:
    TrajRecord(const std::string &traj_folder_path, const std::string &file_name);
    void SavePose(const Eigen::Matrix4f &pose, const double time_stamp);

  private:
    std::ofstream traj_ofs_;
};

class FileManager
{
  public:
    static bool CreateFolder(const std::string &folder_path);
    static bool CreateTxtFile(std::ofstream &ofs, const std::string file_path);
    static bool ClearFolder(const std::string &in_folder_path);
};

class WatchDog
{
  public:
    WatchDog() = delete;
    WatchDog(ros::NodeHandle &nh, const double time_period = 1.0, const double time_max_delay = 10.0);
    ~WatchDog() = default;
    void FeedDog();
    bool GetTimeOutStatus();

  private:
    void TimerCallback(const ros::TimerEvent &event);
    ros::Timer timer_;

    double time_accumulate_ = 0;
    double time_period_ = 0;
    double time_max_delay_ = 0;

    bool time_out_flag_ = false;
};

} // namespace evolving_ad_ns

#endif //_TOOLS_HPP_
