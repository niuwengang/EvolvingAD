#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h> // 包含GNSS消息类型

class FixGnssSub
{
  public:
    FixGnssSub(ros::NodeHandle nh, const std::string topic_name, const size_t buffer_size);

    void ParseData();

  private:
    void MsgCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    ros::Subscriber gnss_sub_;
};
