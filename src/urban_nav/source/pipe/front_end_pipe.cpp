/**
 * @file    preprocerss_pipe.cpp
 * @brief   front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-04
 * @version 0.1.1
 */
#include "front_end_pipe.hpp"

/**
 * @brief front end init
 * @param[in] nh
 * @return
 */
FrontEndPipe::FrontEndPipe(ros::NodeHandle &nh)
{
    /*[1]--params load*/
    paramlist_.package_folder_path = ros::package::getPath("urban_nav");
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/front_end.yaml");

    paramlist_.cloud_sub_topic = config_node["topic_sub"]["cloud_sub_topic"].as<std::string>();

    /*[2]--subscriber and publisher*/
    lidar_odom_pub_ptr_ = std::make_shared<Tools::OdomPub>(nh, "lidar_odom", "map", "lidar");
    cloud_sub_ptr_ = std::make_shared<Tools::CloudSub>(nh, paramlist_.cloud_sub_topic);

    /*[3]--system monitor*/
    log_ptr_ = std::make_shared<Tools::LogRecord>(paramlist_.package_folder_path + "/log", "front_end");
    time_ptr_ = std::make_shared<Tools::TimeRecord>();

    /*[4]--lidar odom*/
    lidar_doom_ptr_ = std::make_shared<LidarOdom>(config_node["lidar_odom"]);

    spdlog::info("frontend_pipe$ inited");
}

/**
 * @brief run
 * @param[in]
 * @return
 */
bool FrontEndPipe::Run()
{
    if (!ReadMsgBuffer())
    {
        return false;
    }

    while (CheckMsgQueue())
    {
        if (!ReadMsg())
        {
            continue;
        }

        lidar_doom_ptr_->InitPose(Eigen::Matrix4f::Identity());
        time_ptr_->Start();
        lidar_doom_ptr_->UpdateOdom(lidar_odom_, cur_cloud_msg_);
        spdlog::info("frontend_pipe$ exec frequence:{}", time_ptr_->End(10e2));

        PublishMsg();
    }

    return true;
}

/**
 * @brief read buffer
 * @param[in]
 * @return
 */
bool FrontEndPipe::ReadMsgBuffer()
{
    cloud_sub_ptr_->ParseData(cloud_msg_queue_);

    return true;
}

/**
 * @brief check queue
 * @param[in]
 * @return
 */
bool FrontEndPipe::CheckMsgQueue()
{
    if (cloud_msg_queue_.size() > 0)
    {
        return true;
    }
    return false;
}

/**
 * @brief read current msg
 * @param[in]
 * @return
 * @note
 */
bool FrontEndPipe::ReadMsg()
{
    cur_cloud_msg_ = cloud_msg_queue_.front();
    cloud_msg_queue_.pop_front();
    return true;
}

/**
 * @brief publish msg
 * @param[in]
 * @return
 */
void FrontEndPipe::PublishMsg()
{
    spdlog::info("frontend_pipe$ queue size:{}", cloud_msg_queue_.size());
    lidar_odom_pub_ptr_->Publish(lidar_odom_, cur_cloud_msg_.time_stamp);
}