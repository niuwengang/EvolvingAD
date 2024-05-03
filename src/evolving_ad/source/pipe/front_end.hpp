/**
 * @file    front_end_pipe.hpp
 * @brief   loc front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version 0.1.4
 */

#ifndef _FRONT_END_HPP_
#define _FRONT_END_HPP_

// yaml
#include <yaml-cpp/yaml.h>
// ros
#include <ros/ros.h>
// topic sub
#include "topic_sub/cloud_sub.hpp"
// topic pub
#include "topic_pub/bbx_pub.hpp"
#include "topic_pub/cloud_pub.hpp"
#include "topic_pub/odom_pub.hpp"
#include "topic_pub/tf_pub.hpp"
// msg
#include "msg/object_msg.hpp"
// module
#include "module/object_detect/object_detect.hpp"
#include "module/odom/lidar_odom.hpp"
// tools
#include "tools/tools.hpp"
// splog
#include <spdlog/spdlog.h>
// thread
#include <thread>
// pcl
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#define DEBUG_DOR

namespace evolving_ad_ns
{
class FrontEndPipe
{
  public:
    FrontEndPipe() = delete;
    FrontEndPipe(ros::NodeHandle &nh, const std::string package_folder_path);
    ~FrontEndPipe() = default;

    bool Run();
    void SendFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex); // send message to other thread
  private:
    void DorPost(const ObjectsMsg &ods_msg, const CloudMsg::CLOUD_PTR &cloud_ptr, CloudMsg::CLOUD_PTR &static_cloud_ptr,
                 CloudMsg::CLOUD_PTR &dynamic_cloud_ptr);

  private:
    /* sub*/
    std::shared_ptr<CloudSub> cloud_sub_ptr_ = nullptr;
    /* pub*/
    std::shared_ptr<CloudPub> static_cloud_pub_ptr_ = nullptr;
    std::shared_ptr<CloudPub> dynamic_cloud_pub_ptr_ = nullptr;
    std::shared_ptr<BbxPub> bbx_pub_ptr_ = nullptr;

    /*variable*/
    std::deque<CloudMsg> cloud_msg_queue_;
    std::deque<Frame> frame_queue_;
    /*tools*/
    std::shared_ptr<TimeRecord> time_record_ptr_ = nullptr;
    /*algorithm module*/
    std::shared_ptr<ObjectDetect> object_detect_ptr_ = nullptr;
    std::shared_ptr<LidarOdom> lidar_odom_ptr_ = nullptr;

    struct ParamList
    {
      public:
        std::string package_folder_path;
        std::string model_file_path;
        std::string cloud_sub_topic;
    } paramlist_;
};
} // namespace evolving_ad_ns

#endif //_FRONT_END_HPP_