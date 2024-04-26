/**
 * @file    back_end_pipe.cpp
 * @brief   back end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-05
 * @version 0.1.1
 */
#ifndef _BACK_END_HPP_
#define _BACK_END_HPP_

// pcl
#include <pcl/common/transforms.h>
// ros
#include <ros/package.h>
#include <ros/ros.h>
// tools--pub
#include "tools/publisher/cloud_pub.hpp"
#include "tools/publisher/odom_pub.hpp"
#include "tools/publisher/path_pub.hpp"
#include "tools/publisher/tf_pub.hpp"
// tools--sub
#include "tools/subscriber/cloud_sub.hpp"
#include "tools/subscriber/imu_sub.hpp"
#include "tools/subscriber/odom_sub.hpp"
// tools--system_monitor
#include "tools/system_monitor/system_monitor.hpp"
// lidar odom
#include "module/pose_graph/pose_graph.hpp"
// yaml lib
#include <yaml-cpp/yaml.h>
// spdlog
#include <spdlog/spdlog.h>

class BackEndPipe
{
  public:
    BackEndPipe() = default;
    ~BackEndPipe() = default;
    BackEndPipe(ros::NodeHandle &nh);
    bool Run();
    bool ReadMsgBuffer();

  private:
    bool CheckMsgQueue();
    bool ReadMsg();
    void PublishMsg();

  private:
    /*topic subscriber and publisher*/
    std::shared_ptr<Tools::CloudSub> cloud_sub_ptr_ = nullptr;
    std::shared_ptr<Tools::OdomSub> gnss_odom_sub_ptr_ = nullptr;
    std::shared_ptr<Tools::OdomSub> lidar_odom_sub_ptr_ = nullptr;

    std::shared_ptr<Tools::CloudPub> cloud_pub_ptr_ = nullptr;
    std::shared_ptr<Tools::OdomPub> lidar_odom_pub_ptr_ = nullptr;
    std::shared_ptr<Tools::TfPub> veh_tf_pub_ptr_ = nullptr;
    std::shared_ptr<Tools::PathPub> path_pub_ptr = nullptr;

    /*variable*/
    std::deque<CloudMsg> cloud_msg_queue_;
    std::deque<PoseMsg> gnss_odom_msg_queue_;
    std::deque<PoseMsg> lidar_odom_msg_queue_;

    CloudMsg cur_cloud_msg_;
    PoseMsg cur_gnss_odom_msg_;
    PoseMsg cur_lidar_odom_msg_;

    Eigen::Matrix4f lidar2gnss_transform_ = Eigen::Matrix4f::Identity();

    /*system monitor*/
    std::shared_ptr<Tools::LogRecord> log_ptr_ = nullptr;
    std::shared_ptr<Tools::TimeRecord> time_ptr_ = nullptr;

    /*algorithm module*/
    std::shared_ptr<PoseGraph> pose_graph_ptr_ = nullptr; // odom pub

    std::deque<PoseMsg> opted_pose_msg_queue_;

    struct ParamLists
    {
      public:
        /*project path*/
        std::string package_folder_path;
        /*topic subscriber*/
        std::string cloud_sub_topic;
        std::string gnss_sub_topic;
        std::string lidar_odom_sub_topic;

    } paramlist_;
};

#endif //_BACK_END_HPP_