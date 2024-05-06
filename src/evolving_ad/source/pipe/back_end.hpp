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

// module
#include "module/calibration/calibration.hpp"
#include "module/graph_optimizer/g2o/g2o_opter.hpp"
#include "module/graph_optimizer/graph_optimizer_interface.hpp"
#include "module/odom/gnss_odom.hpp"
// topic sub
#include "topic_sub/gnss_sub.hpp"
#include "topic_sub/gt_sub.hpp"
// topic pub
#include "topic_pub/bbx_pub.hpp"
#include "topic_pub/cloud_pub.hpp"
#include "topic_pub/odom_pub.hpp"
#include "topic_pub/path_pub.hpp"
#include "topic_pub/tf_pub.hpp"
// tools
#include "tools/tools.hpp"

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
    std::shared_ptr<GtSub> gt_sub_ptr_ = nullptr;
    /*pub*/
    std::shared_ptr<OdomPub> gnss_odom_pub_ptr_ = nullptr;
    std::shared_ptr<OdomPub> gt_odom_pub_ptr_ = nullptr;
    std::shared_ptr<OdomPub> lidar_odom_pub_ptr_ = nullptr;
    std::shared_ptr<CloudPub> cloud_pub_ptr_ = nullptr;
    std::shared_ptr<TfPub> veh_tf_pub_ptr_ = nullptr;
    std::shared_ptr<BbxPub> bbx_pub_ptr_ = nullptr;
    std::shared_ptr<PathPub> path_pub_ptr_ = nullptr;

    /*algorithm module*/
    std::shared_ptr<GnssOdom> gnss_odom_ptr_ = nullptr;
    std::shared_ptr<GnssOdom> gt_odom_ptr_ = nullptr;

    std::shared_ptr<Lidar2GnssCalibration> lidar2gnss_calibration_ptr_ = nullptr;
    std::shared_ptr<GraphOptimizerInterface> graph_optimizer_ptr_ = nullptr;

    /*tools*/
    std::shared_ptr<LogRecord> log_record_ptr_ = nullptr;
    std::shared_ptr<TrajRecord> gt_traj_record_ptr_ = nullptr;
    std::shared_ptr<TrajRecord> my_traj_record_ptr_ = nullptr;
    std::shared_ptr<WatchDog> watchdog_ptr_ = nullptr;

    /*value*/
    std::deque<Frame> frame_queue_;
    std::deque<GnssMsg> gnss_msg_queue_;
    std::deque<GnssMsg> gt_msg_queue_;
    std::deque<Frame> keyframe_queue_;
    std::deque<Eigen::Isometry3d> opted_pose_queue_;
    Eigen::Matrix4f T_gnss2lidar_ = Eigen::Matrix4f::Identity();
    bool online_calibration_flag_ = false;
    Eigen::Matrix4f last_keyframe_pose_ = Eigen::Matrix4f::Identity();

    struct ParamList
    {
      public:
        std::string package_folder_path;
        std::string result_folder_path;

        std::string gnss_sub_topic;
        std::string gt_sub_topic;

    } paramlist_;
};
} // namespace evolving_ad_ns
#endif