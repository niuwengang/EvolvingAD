/**
 * @file    front_end_pipe.cpp
 * @brief   front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-04
 * @version 0.1.1
 */

#ifndef _FRONT_END_HPP_
#define _FRONT_END_HPP_
// ros
#include <ros/package.h>
#include <ros/ros.h>
// tools--sub
#include "tools/subscriber/cloud_sub.hpp"
#include "tools/subscriber/gnss_sub.hpp"
#include "tools/subscriber/imu_sub.hpp"
// tools--pub
#include "tools/publisher/cloud_pub.hpp"
#include "tools/publisher/gnss_pub.hpp"
#include "tools/publisher/imu_pub.hpp"
#include "tools/publisher/odom_pub.hpp"
// tools--system_monitor
#include "tools/system_monitor/system_monitor.hpp"
// yaml
#include <yaml-cpp/yaml.h>
// algorithm module
#include "module/lidar_odom/lidar_odom.cpp"

class FrontEndPipe
{
  public:
    FrontEndPipe() = delete;
    FrontEndPipe(ros::NodeHandle &nh); // must init wiith node handle
    ~FrontEndPipe() = default;
    bool Run();

  private:
    bool ReadMsgBuffer();
    bool CheckMsgQueue();
    bool ReadMsg();
    void PublishMsg();

  private:
    /*topic subscriber and publisher*/
    std::shared_ptr<Tools::CloudSub> cloud_sub_ptr_ = nullptr;
    std::shared_ptr<Tools::OdomPub> lidar_odom_pub_ptr_ = nullptr;

    /*variable*/
    std::deque<CloudMsg> cloud_msg_queue_;
    CloudMsg cur_cloud_msg_;

    /*system monitor*/
    std::shared_ptr<Tools::LogRecord> log_ptr_ = nullptr;
    std::shared_ptr<Tools::TimeRecord> time_ptr_ = nullptr;

    /*algorithm module*/
    std::shared_ptr<LidarOdom> lidar_doom_ptr_ = nullptr;
    Eigen::Matrix4f lidar_odom_ = Eigen::Matrix4f::Identity();

    /*paramlist*/
    struct ParamLists
    {
      public:
        /*project path*/
        std::string package_folder_path;
        /*topic name*/
        std::string cloud_sub_topic;
    } paramlist_;
};

#endif //_FRONT_END_HPP_