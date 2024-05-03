/**
 * @file    front_node.cpp
 * @brief   veh loc node for autopilot
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version v0.1.4
 */

// ros
#include <ros/package.h>
#include <ros/ros.h>
// thread
#include <thread>
// pipe
#include "pipe/back_end.hpp"
#include "pipe/front_end.hpp"

namespace evolving_ad_ns
{

class Scheduler
{

  public:
    void FrontEndThread()
    {
        ros::Rate rate_front_end(10);
        std::shared_ptr<FrontEndPipe> front_end_ptr = std::make_shared<FrontEndPipe>(nh1_, package_folder_path_);

        while (ros::ok())
        {
            front_end_ptr->Run();
            front_end_ptr->SendFrameQueue(frame_queue_, scheduler_mutex_);
            rate_front_end.sleep();
        }
    }
    void BackEndThread()
    {
        ros::Rate rate_back_end(100);
        std::shared_ptr<BackEndPipe> back_end_ptr = std::make_shared<BackEndPipe>(nh2_, package_folder_path_);

        while (ros::ok())
        {
            back_end_ptr->ReveiveFrameQueue(frame_queue_, scheduler_mutex_);
            back_end_ptr->Run();
            rate_back_end.sleep();
        }
    }

    Scheduler(ros::NodeHandle &nh1, ros::NodeHandle &nh2)
    {
        nh1_ = nh1;
        nh2_ = nh2;
        package_folder_path_ = ros::package::getPath("evolving_ad");
        std::thread Thread1(&Scheduler::FrontEndThread, this);
        std::thread Thread2(&Scheduler::BackEndThread, this);

        Thread1.detach();
        Thread2.detach();
    }
    ~Scheduler()
    {
    }

  private:
    ros::NodeHandle nh1_, nh2_;
    std::string package_folder_path_;
    std::deque<Frame> frame_queue_;
    std::mutex scheduler_mutex_;
};
} // namespace evolving_ad_ns

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ad_node");
    ros::NodeHandle nh1, nh2;
    ros::Rate rate_main_thread(100);

    std::shared_ptr<evolving_ad_ns::Scheduler> scheduler_ptr = std::make_shared<evolving_ad_ns::Scheduler>(nh1, nh2);

    while (ros::ok())
    {
        ros::spinOnce();
        rate_main_thread.sleep();
    }
    return 0;
}
