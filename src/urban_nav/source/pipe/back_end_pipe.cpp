/**
 * @file    backend_pipe.cpp
 * @brief   backend pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-04
 * @version 0.1.1
 */
#include "back_end_pipe.hpp"

/**
 * @brief back end init
 * @param[in] nh
 * @return
 */
BackEndPipe::BackEndPipe(ros::NodeHandle &nh)
{
    /*[1]--load param*/
    paramlist_.package_folder_path = ros::package::getPath("urban_nav");
    YAML::Node config = YAML::LoadFile(paramlist_.package_folder_path + "/config/back_end.yaml");

    paramlist_.cloud_sub_topic = config["topic_sub"]["cloud_sub_topic"].as<std::string>();
    paramlist_.gnss_sub_topic = config["topic_sub"]["gnss_sub_topic"].as<std::string>();
    paramlist_.lidar_odom_sub_topic = config["topic_sub"]["lidar_odom_sub_topic"].as<std::string>();

    /*[2]--topic subscriber and publisher*/
    lidar_odom_sub_ptr_ = std::make_shared<Tools::OdomSub>(nh, paramlist_.lidar_odom_sub_topic);
    gnss_sub_ptr_ = std::make_shared<Tools::OdomSub>(nh, paramlist_.gnss_sub_topic);
    cloud_sub_ptr_ = std::make_shared<Tools::CloudSub>(nh, paramlist_.cloud_sub_topic);

    fusion_odom_pub_ptr_ = std::make_shared<Tools::OdomPub>(nh, "fusion_odom", "map", "veh");
    veh_tf_pub_ptr_ = std::make_shared<Tools::TfPub>("map", "ground_link"); // tf tree

    /*[3]--system monitor*/
    log_ptr_ = std::make_shared<Tools::LogRecord>(paramlist_.package_folder_path + "/log", "back_end");
    time_ptr_ = std::make_shared<Tools::TimeRecord>();

    /*[4]--algorithm  module init*/
    pose_graph_ptr_ = std::make_shared<PoseGraph>(); // todo add yaml node
}

/**
 * @brief run
 * @param[in]
 * @return
 */
bool BackEndPipe::Run()
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

        static bool lidar2gnss_transform_init_flag = false;
        if (lidar2gnss_transform_init_flag == false)
        {
            lidar2gnss_transform_ = cur_gnss_odom_msg_.pose * cur_lidar_odom_msg_.pose.inverse();
            lidar2gnss_transform_init_flag = true;
        }
        cur_lidar_odom_msg_.pose =
            lidar2gnss_transform_ * cur_lidar_odom_msg_.pose; // lidar coordinate align to gnss coordinate

        pose_graph_ptr_->UpdatePose(cur_gnss_odom_msg_, cur_lidar_odom_msg_, fusion_odom_msg_);

        // spdlog::info("backend_node$ core exec hz:{}", cur_cloud_msg_.time_stamp);

        PublishMsg();
    }

    return true;
}

/**
 * @brief read message
 * @param[in]
 * @return
 */
bool BackEndPipe::ReadMsgBuffer()
{
    cloud_sub_ptr_->ParseData(cloud_msg_queue_);
    gnss_sub_ptr_->ParseData(gnss_odom_msg_queue_);
    lidar_odom_sub_ptr_->ParseData(lidar_odom_msg_queue_);

    return true;
}

/**
 * @brief check message
 * @param[in]
 * @return
 */
bool BackEndPipe::CheckMsgQueue()
{
    if (cloud_msg_queue_.size() == 0)
    {
        return false;
    }
    if (gnss_odom_msg_queue_.size() == 0)
    {
        return false;
    }
    if (lidar_odom_msg_queue_.size() == 0)
    {
        return false;
    }
    return true;
}

/**
 * @brief read message
 * @param[in]
 * @return
 * @note
 */
bool BackEndPipe::ReadMsg()
{
    cur_cloud_msg_ = cloud_msg_queue_.front();
    cur_gnss_odom_msg_ = gnss_odom_msg_queue_.front();
    cur_lidar_odom_msg_ = lidar_odom_msg_queue_.front();

    const double refer_time =
        std::max(std::max(cur_cloud_msg_.time_stamp, cur_gnss_odom_msg_.time_stamp), cur_lidar_odom_msg_.time_stamp);

    const double cloud_refer_timediff = fabs(refer_time - cur_cloud_msg_.time_stamp);
    const double gnss_refer_timediff = fabs(refer_time - cur_gnss_odom_msg_.time_stamp);
    const double lidar_odom_refer_timediff = fabs(refer_time - cur_lidar_odom_msg_.time_stamp);

    if (cloud_refer_timediff < 0.05 and gnss_refer_timediff < 0.05 and lidar_odom_refer_timediff < 0.05)
    {
        cloud_msg_queue_.pop_front();
        gnss_odom_msg_queue_.pop_front();
        lidar_odom_msg_queue_.pop_front();
        return true;
    }
    else
    {
        if (cloud_refer_timediff > 0.05)
        {
            cloud_msg_queue_.pop_front();
        }
        if (gnss_refer_timediff > 0.05)
        {
            gnss_odom_msg_queue_.pop_front();
        }
        if (lidar_odom_refer_timediff > 0.05)
        {
            lidar_odom_msg_queue_.pop_front();
        }
        return false;
    }
}

/**
 * @brief publish message
 * @param[in]
 * @return
 */
void BackEndPipe::PublishMsg()
{

    fusion_odom_pub_ptr_->Publish(fusion_odom_msg_.pose, fusion_odom_msg_.time_stamp);
    veh_tf_pub_ptr_->SendTransform(fusion_odom_msg_.pose);

    // /*pub*/
    // CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
    // Eigen::Matrix4f transforme_matrix = fusion_odom_msg_.pose;
    // transforme_matrix(2, 3) += 1.0; // only view1
    // pcl::transformPointCloud(*cur_cloud_msg_.cloud_ptr, *transformed_cloud_ptr, transforme_matrix);
    // cloud_pub_ptr_->Publish(transformed_cloud_ptr, fusion_odom_msg_.time_stamp);
}

// /**
//  * @brief 轨迹保存
//  * @param[in]
//  * @return
//  */
// void BackEndPipe::SaveTrajectory(const Eigen::Matrix4f &target_odom, const std::string file_name)
// {
//     static std::ofstream target_odom_stream;
//     static bool is_file_created = false;
//     /*1--创建文件*/
//     if (!is_file_created)
//     {
//         std::string file_path = paramlist_.package_folder_path + "/result/" + file_name + ".txt";
//         Tools::FileManager::CreateTxtFile(target_odom_stream, traj_file_path)

//             is_file_created = true;
//     }
//     /*2--写入轨迹*/
//     for (int i = 0; i < 3; ++i)
//     {
//         for (int j = 0; j < 4; ++j)
//         {
//             target_odom_stream << target_odom(i, j);
//             if (i == 2 && j == 3)
//             {
//                 target_odom_stream << std::endl;
//             }
//             else
//             {
//                 target_odom_stream << " ";
//             }
//         }
//     }
// }