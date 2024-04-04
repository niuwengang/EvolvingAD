/**
 * @file    odom_pipe.hpp
 * @brief   里程计pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-04
 * @version 0.1.0
 */

#ifndef _ODOM_PIPE_HPP_
#define _ODOM_PIPE_HPP_

// ros lib
#include <ros/package.h>
#include <ros/ros.h>
// eigen lib
#include <Eigen/Core>
// tools lib
#include "tools/file_manager/file_manager.hpp"
#include "tools/monitor/monitor.hpp"
#include "tools/parameter/parameter.hpp"

#include "tools/publisher/cloud_pub.hpp"
#include "tools/publisher/gnss_pub.hpp"
#include "tools/publisher/imu_pub.hpp"
#include "tools/publisher/odom_pub.hpp"
#include "tools/publisher/tf_pub.hpp"

#include "tools/subscriber/cloud_sub.hpp"
#include "tools/subscriber/gnss_sub.hpp"
#include "tools/subscriber/imu_sub.hpp"

// yaml lib
#include <yaml-cpp/yaml.h>
// module
#include "module/cloud_filter/cloud_filter_interface.hpp"
#include "module/cloud_filter/voxel_filter.hpp"

#include "module/cloud_registration/cloud_registration_interface.hpp"
#include "module/cloud_registration/fast_gicp_registration.hpp"
#include "module/cloud_registration/ndt_registration.hpp"
// msg lib
#include "user_msg/frame_msg.hpp"
#include "user_msg/imu_msg.hpp"
// pcl lib
#include <pcl/common/transforms.h>
// omp lib
#include "omp.h"

class OdomPipe
{

  public:
    OdomPipe() = default;
    OdomPipe(ros::NodeHandle &nh);
    ~OdomPipe() = default;
    /*功能函数*/
    void Run();        // 运行
    bool ReadBuffer(); // 读取缓冲区
    bool ReadMsg();    // 读取当前消息

    bool UpdateLidarOdom(const CloudMsg &cloud_msg, Eigen::Matrix4f &cloud_pose); // 更新雷达里程计
    bool UpdateLocalMap(const Frame &new_key_frame);                              // 更新局部地图
    bool InitLidarOdom();                                                         // 初始化雷达里程计

    bool InitGnssOdom();                             // 初始化gnss里程计
    bool UpdateGnssOdom(Eigen::Matrix4f &gnss_pose); // 更新gnss里程计

    void SaveTrajectory(const Eigen::Matrix4f &lidar_odom, const Eigen::Matrix4f &gnss_odom); // 轨迹保存

  private:
    /*tools指针*/
    std::shared_ptr<Tools::ParamList> paramlist_ptr_ = nullptr; // 参数管理
    std::shared_ptr<Tools::TimeRecord> time_ptr_ = nullptr;     // 时间统计
    std::shared_ptr<Tools::LogRecord> log_ptr_ = nullptr;       // 日志管理

    std::shared_ptr<Tools::ImuSub> imu_sub_ptr_ = nullptr; // imu消息订阅
    std::shared_ptr<Tools::ImuPub> imu_pub_ptr_ = nullptr; // imu消息发布

    std::shared_ptr<Tools::CloudSub> cloud_sub_ptr_ = nullptr; // 点云接收
    std::shared_ptr<Tools::CloudPub> cloud_pub_ptr_ = nullptr; // 点云发布

    std::shared_ptr<Tools::GnssSub> gnss_sub_ptr_ = nullptr; // gnss接收
    std::shared_ptr<Tools::GnssPub> gnss_pub_ptr_ = nullptr; // gnss发布

    std::shared_ptr<Tools::OdomPub> body_odom_pub_ptr_ = nullptr;  // 融合里程计发布
    std::shared_ptr<Tools::OdomPub> gnss_odom_pub_ptr_ = nullptr;  // gnss里程计发布
    std::shared_ptr<Tools::OdomPub> lidar_odom_pub_ptr_ = nullptr; // 激光里程计发布

    std::shared_ptr<Tools::TfPub> body_tf_pub_ptr_ = nullptr; // 车身里程计tf发布

    /*里程计*/
    Eigen::Matrix4f lidar_odom_ = Eigen::Matrix4f::Identity(); // 雷达里程计
    Eigen::Matrix4f body_odom_ = Eigen::Matrix4f::Identity();  // 车体里程计
    Eigen::Matrix4f gnss_odom_ = Eigen::Matrix4f::Identity();  // gnss里程计

    /*传感器消息*/
    std::deque<ImuMsg> imu_msg_queue_;     // imu消息序列
    std::deque<CloudMsg> cloud_msg_queue_; // cloud序列
    std::deque<GnssMsg> gnss_msg_queue_;   // gnss序列

    ImuMsg cur_imu_msg_;     // 当前imu消息
    CloudMsg cur_cloud_msg_; // 当前点云消息
    GnssMsg cur_gnss_msg_;   // 当前gnss消息

    Eigen::Matrix4f lidar_odom_init_pose_ = Eigen::Matrix4f::Identity(); // 雷达里程计初始位姿

    /*module*/
    std::shared_ptr<Module::CloudFilterInterface> single_scan_filter_ptr_ = nullptr; // 当前单帧滤波
    std::shared_ptr<Module::CloudFilterInterface> local_map_filter_ptr_ = nullptr;   // 局部地图滤波
    std::shared_ptr<Module::CloudFilterInterface> global_map_filter_ptr_ = nullptr;  // 全局地图滤波

    std::shared_ptr<Module::CloudRegistrationInterface> registration_ptr_ = nullptr; // 点云配准

    /*点云地图*/
    std::deque<Frame> local_map_frames_;  // 局部地图关键帧序列
    std::deque<Frame> global_map_frames_; // 全局关键帧地图序列

    CloudMsg::CLOUD_PTR single_scan_ptr_; // 当前单帧指针
    CloudMsg::CLOUD_PTR local_map_ptr_;   // 局部点云指针
    CloudMsg::CLOUD_PTR global_map_ptr_;  // 全局地图指针
};

#endif