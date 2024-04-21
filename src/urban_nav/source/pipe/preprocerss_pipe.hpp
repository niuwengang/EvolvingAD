/**
 * @file    preprocerss_pipe.cpp
 * @brief   preprocerss pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-09 update
 * @version 0.1.1
 */
#ifndef _PREPROCESS_PIPE_HPP_
#define _PREPROCESS_PIPE_HPP_

// ros
#include <ros/package.h>
#include <ros/ros.h>
// tools -- sub
#include "tools/subscriber/cloud_sub.hpp"
#include "tools/subscriber/gnss_sub.hpp"
#include "tools/subscriber/imu_sub.hpp"
// tools --sub
#include "tools/publisher/cloud_pub.hpp"
#include "tools/publisher/gnss_pub.hpp"
#include "tools/publisher/imu_pub.hpp"
#include "tools/publisher/odom_pub.hpp"
#include "tools/publisher/tf_pub.hpp"
// tools -- system_monitor
#include "tools/system_monitor/system_monitor.hpp"
// yaml
#include <yaml-cpp/yaml.h>
// algorithm module
#include "module/gnss_odom/gnss_odom.hpp"
#include "module/object_detection/object_detection.hpp"

#include "thirdpartylib/ground_seg/include/dipgseg.h"
// openmp
#include <omp.h>

class PreProcessPipe
{
  public:
    PreProcessPipe() = delete;
    PreProcessPipe(ros::NodeHandle &nh);
    ~PreProcessPipe() = default;
    bool Run();

  private:
    bool ReadMsgBuffer();
    bool CheckMsgQueue();
    bool ReadMsg();
    void PublishMsg();

  private:
    /*topic sub and pub*/
    std::shared_ptr<Tools::ImuSub> imu_sub_ptr_ = nullptr;
    std::shared_ptr<Tools::CloudSub> cloud_sub_ptr_ = nullptr;
    std::shared_ptr<Tools::GnssSub> gnss_sub_ptr_ = nullptr;

    std::shared_ptr<Tools::ImuPub> imu_pub_ptr_ = nullptr;
    std::shared_ptr<Tools::CloudPub> cloud_pub_ptr_ = nullptr;
    std::shared_ptr<Tools::OdomPub> gnss_pub_ptr_ = nullptr;
    std::shared_ptr<Tools::BbxPub> bbx_pub_ptr_ = nullptr;
    // std::shared_ptr<Tools::TfPub> veh_tf_pub_ptr_ = nullptr;

    /*sensor queue and current*/
    std::deque<ImuMsg> imu_msg_queue_;
    std::deque<CloudMsg> cloud_msg_queue_;
    std::deque<GnssMsg> gnss_msg_queue_;
    OdsMsg ods_msg_;

    ImuMsg cur_imu_msg_;
    CloudMsg cur_cloud_msg_;
    GnssMsg cur_gnss_msg_;

    /*gnss odom*/
    Eigen::Matrix4f gnss_odom_ = Eigen::Matrix4f::Identity(); // gnss odom
    std::shared_ptr<GnssOdom> gnss_odom_ptr_ = nullptr;       // gnss odom algorithm

    /*system monitor*/
    std::shared_ptr<Tools::LogRecord> log_ptr_ = nullptr;
    std::shared_ptr<Tools::TimeRecord> time_ptr_ = nullptr;

    /*algorithm module*/
    std::shared_ptr<ObjectDetection> object_detection_ptr_ = nullptr;
    std::shared_ptr<DIPGSEG::Dipgseg> ground_seg_ptr_ = nullptr;

    /*paramlist*/
    struct ParamList
    {
      public:
        /*project path*/
        std::string package_folder_path;
        /*topic subscriber*/
        std::string imu_sub_topic;
        std::string cloud_sub_topic;
        std::string gnss_sub_topic;
        /*extrinsic*/
        Eigen::Matrix4f lidar_to_body = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f imu_to_body = Eigen::Matrix4f::Identity();
        std::string model_file_path; // infer model

    } paramlist_;
};

#endif //  _PREPROCESS_PIPE_HPP_