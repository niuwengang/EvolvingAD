/**
 * @file    imu_sub.hpp
 * @brief   imu消息订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

// user msg
#include "user_msg/imu_msg.hpp"
// c++ lib
#include <deque>
#include <mutex>
// ros lib
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>

namespace Tools
{
class ImuSub
{
  public:
    ImuSub() = default;
    ImuSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 1000000);
    ~ImuSub() = default;
    void ParseData(std::deque<ImuMsg> &imu_msg_queue);

  private:
    void MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    std::deque<ImuMsg> imu_msg_queue_;

    std::mutex mutex_;
};
} // namespace Tools