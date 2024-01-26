#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuSub
{
  public:
    ImuSub(ros::NodeHandle nh, const std::string topic_name, const size_t buffer_size);

    void ParseData();

  private:
    void Callback(const sensor_msgs::Imu::ConstPtr &msg);

    ros::Subscriber imu_sub_;
};