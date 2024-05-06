#include "back_end.hpp"

namespace evolving_ad_ns
{
BackEndPipe::BackEndPipe(ros::NodeHandle &nh, const std::string package_folder_path)
{
    /*[1]--load params*/
    paramlist_.package_folder_path = package_folder_path;
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/ad.yaml");
    FileManager::ClearFolder(paramlist_.package_folder_path + "/result"); // reset folder

    paramlist_.gnss_sub_topic = config_node["topic_sub"]["gnss_sub_topic"].as<std::string>();
    paramlist_.gt_sub_topic = config_node["topic_sub"]["gt_sub_topic"].as<std::string>();

    /*[2]--sub*/
    gnss_sub_ptr_ = std::make_shared<GnssSub>(nh, paramlist_.gnss_sub_topic);
    gt_sub_ptr_ = std::make_shared<GtSub>(nh, paramlist_.gt_sub_topic);

    /*[3]--pub*/
    /*a--odom*/
    lidar_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_lidar_odom", "map", "lidar");
    gnss_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_gnss_odom", "map", "gnss");
    gt_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_gnss_gt", "map", "gt");
    /*b--others*/
    cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "back_end_cloud", "map");
    veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "back_end_bbx", "map");

    /*[4]--algorithm module*/
    gnss_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
    gt_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
    lidar2gnss_calibration_ptr_ = std::make_shared<Lidar2GnssCalibration>(0.1, 20);

    /*[5]--tools*/
    log_record_ptr_ = std::make_shared<LogRecord>(paramlist_.package_folder_path + "/log", "back_end");
    gt_traj_record_ptr_ = std::make_shared<TrajRecord>(paramlist_.package_folder_path + "/result/traj", "gt");
    gnss_traj_record_ptr_ = std::make_shared<TrajRecord>(paramlist_.package_folder_path + "/result/traj", "gnss");
}

bool BackEndPipe::Run()
{

    gnss_sub_ptr_->ParseData(gnss_msg_queue_);
    if (frame_queue_.size() > 0)
    {
        frame_ = frame_queue_.front();
        frame_queue_.pop_front();

        if (GnssMsg::TimeSync(gnss_msg_queue_, gnss_msg_, frame_.time_stamp))
        {
            gnss_odom_ptr_->InitPose(gnss_msg_);
            gt_odom_ptr_->InitPose(gnss_msg_);
            Eigen::Matrix4f gnss_pose = Eigen::Matrix4f::Identity();
            gnss_odom_ptr_->ComputePose(gnss_msg_, gnss_pose);

            if (online_calibration_flag_ == false)
            {
                online_calibration_flag_ = lidar2gnss_calibration_ptr_->Calibration(
                    gnss_pose.block<3, 1>(0, 3), frame_.pose.block<3, 1>(0, 3), T_gnss2lidar_);
            }

            gnss_odom_pub_ptr_->Publish(gnss_pose);
        }
        if (online_calibration_flag_ == true)
        {
            frame_.pose = T_gnss2lidar_ * frame_.pose;

            /*a--display odom*/
            lidar_odom_pub_ptr_->Publish(frame_.pose);
            veh_tf_pub_ptr_->SendTransform(frame_.pose);

            /*b--display cloud*/
            CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
            pcl::transformPointCloud(*frame_.cloud_msg.cloud_ptr, *transformed_cloud_ptr, frame_.pose);
            cloud_pub_ptr_->Publish(transformed_cloud_ptr, 0.0);

            /*b--display object*/
            for (auto &object : frame_.objects_msg.objects_vec)
            {
                Eigen::Matrix4f object_pose;
                object_pose.block<3, 1>(0, 3) << object.x, object.y, object.z;
                object_pose.block<3, 3>(0, 0) = object.q.toRotationMatrix();

                object_pose = frame_.pose * object_pose; // has been aligned to gnss

                object.x = object_pose(0, 3);
                object.y = object_pose(1, 3);
                object.z = object_pose(2, 3);
                object.q = Eigen::Quaternionf(object_pose.block<3, 3>(0, 0));
            }
            bbx_pub_ptr_->Publish(frame_.objects_msg);
        }
    }

    gt_sub_ptr_->ParseData(gt_msg_queue_);
    if (gt_msg_queue_.size() > 0 && gnss_odom_ptr_->GetInitStatus())
    {
        gt_msg_ = gt_msg_queue_.front();
        gt_msg_queue_.pop_front();

        Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
        gnss_odom_ptr_->ComputePose(gt_msg_, gt_pose);

        log_record_ptr_->file_->info("gt timestamp:{}", gt_msg_.time_stamp);

        gt_odom_pub_ptr_->Publish(gt_pose, gt_msg_.time_stamp);

        // gt_traj_record_ptr_->SavePose(gt_pose, gt_msg_.time_stamp);
    }

    return true;
}

void BackEndPipe::ReveiveFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex)
{
    mutex.lock();
    if (!frame_queue.empty())
    {
        for (size_t i = 0; i < frame_queue.size(); i++)
        {
            Frame frame = frame_queue.at(i);

            frame_queue_.push_back(frame);
        }
        frame_queue.clear();
    }
    mutex.unlock();
}

} // namespace evolving_ad_ns