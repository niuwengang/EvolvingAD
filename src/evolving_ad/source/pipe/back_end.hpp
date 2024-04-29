#ifndef _BACK_END_HPP_
#define _BACK_END_HPP_

// yaml
#include <yaml-cpp/yaml.h>
// ros
#include <ros/ros.h>
// c++
#include "msg/frame.hpp"
#include <deque>

// pcl
#include <pcl/common/transforms.h>

#include "module/odom/gnss_odom.hpp"
// topic sub
#include "topic_sub/gnss_sub.hpp"
// topic pub
#include "topic_pub/bbx_pub.hpp"
#include "topic_pub/cloud_pub.hpp"
#include "topic_pub/odom_pub.hpp"
#include "topic_pub/tf_pub.hpp"

namespace evolving_ad_ns
{

class BackEndPipe
{
  public:
    BackEndPipe() = delete;
    BackEndPipe(ros::NodeHandle &nh, const std::string package_folder_path);
    ~BackEndPipe() = default;
    bool Run();
    void ReveiveFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex);

  private:
    /*sub*/
    std::shared_ptr<GnssSub> gnss_sub_ptr_ = nullptr;
    /*pub*/
    std::shared_ptr<OdomPub> gnss_odom_pub_ptr_ = nullptr;
    std::shared_ptr<OdomPub> lidar_odom_pub_ptr_ = nullptr;
    std::shared_ptr<CloudPub> cloud_pub_ptr_ = nullptr;
    std::shared_ptr<TfPub> veh_tf_pub_ptr_ = nullptr;
    std::shared_ptr<BbxPub> bbx_pub_ptr_ = nullptr;

    /*algorithm module*/
    std::shared_ptr<GnssOdom> gnss_odom_ptr_ = nullptr;

    std::deque<Frame> frame_queue_;
    std::deque<GnssMsg> gnss_msg_queue_;

    struct ParamList
    {
      public:
        std::string package_folder_path;
        std::string gnss_sub_topic;
    } paramlist_;
};
} // namespace evolving_ad_ns
#endif