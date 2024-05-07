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
    path_pub_ptr_ = std::make_shared<PathPub>(nh, "opt_path", "map");
    /*b--others*/
    cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "back_end_cloud", "map");
    veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "back_end_bbx", "map");

    /*[4]--algorithm module*/
    gnss_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
    gt_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
    lidar2gnss_calibration_ptr_ = std::make_shared<Lidar2GnssCalibration>(0.05, 30);
    graph_optimizer_ptr_ = std::make_shared<G2oOpter>("lm_var");

    /*[5]--tools*/
    log_record_ptr_ = std::make_shared<LogRecord>(paramlist_.package_folder_path + "/log", "back_end");
    gt_traj_record_ptr_ = std::make_shared<TrajRecord>(paramlist_.package_folder_path + "/result/traj", "gt");
    my_traj_record_ptr_ = std::make_shared<TrajRecord>(paramlist_.package_folder_path + "/result/traj", "my");
    watchdog_ptr_ = std::make_shared<WatchDog>(nh, 1.0, 30);
}

bool BackEndPipe::Run()
{

    /*[1]-read msg*/
    bool frame_update_flag = false, gnss_update_flag = false, gt_update_flag = false;
    Eigen::Matrix4f gnss_pose = Eigen::Matrix4f::Identity();

    Frame frame;
    GnssMsg gnss_msg;
    GnssMsg gt_msg;

    if (!frame_queue_.empty())
    {
        frame = frame_queue_.front();
        frame_queue_.pop_front();
        frame_update_flag = true;
    }

    gnss_sub_ptr_->ParseData(gnss_msg_queue_);
    if (!gnss_msg_queue_.empty())
    {
        gnss_update_flag = GnssMsg::TimeSync(gnss_msg_queue_, gnss_msg, frame.time_stamp);
    }

    gt_sub_ptr_->ParseData(gt_msg_queue_);
    if (!gt_msg_queue_.empty())
    {
        gt_msg = gt_msg_queue_.front();
        gt_msg_queue_.pop_front();
        gt_update_flag = true;
    }

    /*[2]-handle gnss*/
    if (gnss_update_flag == true)
    {
        gnss_odom_ptr_->InitPose(gnss_msg);
        gt_odom_ptr_->InitPose(gnss_msg);
        gnss_odom_ptr_->ComputePose(gnss_msg, gnss_pose);

        gnss_odom_pub_ptr_->Publish(gnss_pose);
        // gnss_traj_record_ptr_->SavePose(gnss_pose, gnss_msg_.time_stamp);
    }

    /*[3]-handle gt (relay on gnss handle)*/
    if (gt_update_flag == true and gt_odom_ptr_->GetInitStatus() == true)
    {

        Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
        gnss_odom_ptr_->ComputePose(gt_msg, gt_pose);

        gt_odom_pub_ptr_->Publish(gt_pose, gt_msg.time_stamp);
        gt_traj_record_ptr_->SavePose(gt_pose, gt_msg.time_stamp);
    }

    /*[4]--online_calibration_flag_ relay on gnss handle)*/
    if (online_calibration_flag_ == false and gnss_update_flag == true and frame_update_flag == true)
    {
        online_calibration_flag_ = lidar2gnss_calibration_ptr_->Calibration(
            gnss_pose.block<3, 1>(0, 3), frame.pose.block<3, 1>(0, 3), T_gnss2lidar_);
        if (online_calibration_flag_ == true)
        {
            log_record_ptr_->terminal_->warn("online_calibration ok");
        }
    }

    /*[5]-handle gt*/
    if (online_calibration_flag_ == true and frame_update_flag == true)
    {
        frame.pose = T_gnss2lidar_ * frame.pose;

        lidar_odom_pub_ptr_->Publish(frame.pose);
        veh_tf_pub_ptr_->SendTransform(frame.pose);

        /*a--opt*/
        if (sqrt(pow(frame.pose(0, 3) - last_keyframe_pose_(0, 3), 2) +
                 pow(frame.pose(1, 3) - last_keyframe_pose_(1, 3), 2) +
                 pow(frame.pose(2, 3) - last_keyframe_pose_(2, 3), 2)) >= 2.0 or
            keyframe_queue_.size() == 0)
        {
            frame.index = static_cast<unsigned int>(keyframe_queue_.size());
            keyframe_queue_.push_back(frame); // add to keyframe queue

            /*add vertex*/
            Eigen::Isometry3d isometry;
            isometry.matrix() = frame.pose.cast<double>();
            if (graph_optimizer_ptr_->GetOptNodeNum() == 0) // first must be fixed
            {
                graph_optimizer_ptr_->AddSe3Vertex(isometry, true);
            }
            else
            {
                graph_optimizer_ptr_->AddSe3Vertex(isometry, false);
            }
            /*add inter edge*/
            unsigned int node_num = graph_optimizer_ptr_->GetOptNodeNum();
            if (node_num >= 2)
            {
                Eigen::Matrix4f relative_pose = last_keyframe_pose_.inverse() * frame.pose;
                isometry.matrix() = relative_pose.cast<double>();
                std::array<double, 6> noise_array = {0.5, 0.5, 0.5, 0.001, 0.001, 0.001};
                graph_optimizer_ptr_->AddInteriorSe3Edge(node_num - 2, node_num - 1, isometry, noise_array);
            }
            /*add gps edge*/
            if (gnss_update_flag == true)
            {

                if (sqrt(pow(last_gnss_pose_(0, 3) - gnss_pose(0, 3), 2) +
                         pow(last_gnss_pose_(1, 3) - gnss_pose(1, 3), 2)) >= 1)
                {
                    Eigen::Vector3d xyz(static_cast<double>(gnss_pose(0, 3)), static_cast<double>(gnss_pose(1, 3)),
                                        static_cast<double>(gnss_pose(2, 3)));
                    std::array<double, 3> noise_array = {5.0, 5.0, 5.0};
                    graph_optimizer_ptr_->AddPriorXYZEdge(node_num - 1, xyz, noise_array);
                    last_gnss_pose_ = gnss_pose;
                }
            }

            if (graph_optimizer_ptr_->Opimtize() == true)
            {
                graph_optimizer_ptr_->GetOptPoseQueue(opted_pose_queue_);
                path_pub_ptr_->Publish(opted_pose_queue_);
            }

            last_keyframe_pose_ = frame.pose; // record last pose
        }
    }
    /*[6]--*/
    if (frame_update_flag == true or gnss_update_flag == true)
    {
        watchdog_ptr_->FeedDog();
    }
    if (watchdog_ptr_->GetTimeOutStatus() == true)
    {
        if (opted_pose_queue_.size() != keyframe_queue_.size())
        {
            log_record_ptr_->terminal_->error("not same nums");
        }
        for (int i = 0; i < opted_pose_queue_.size(); i++)
        {
            my_traj_record_ptr_->SavePose(opted_pose_queue_.at(i).matrix().cast<float>(),
                                          keyframe_queue_.at(i).time_stamp);
        }
        log_record_ptr_->terminal_->info("has save my traj");
        exit(EXIT_SUCCESS);
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

// /*b--display cloud*/
// CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
// pcl::transformPointCloud(*frame_.cloud_msg.cloud_ptr, *transformed_cloud_ptr, frame_.pose);
// cloud_pub_ptr_->Publish(transformed_cloud_ptr, 0.0);

// /*b--display object*/
// for (auto &object : frame_.objects_msg.objects_vec)
// {
//     Eigen::Matrix4f object_pose;
//     object_pose.block<3, 1>(0, 3) << object.x, object.y, object.z;
//     object_pose.block<3, 3>(0, 0) = object.q.toRotationMatrix();

//     object_pose = frame_.pose * object_pose; // has been aligned to gnss

//     object.x = object_pose(0, 3);
//     object.y = object_pose(1, 3);
//     object.z = object_pose(2, 3);
//     object.q = Eigen::Quaternionf(object_pose.block<3, 3>(0, 0));
// }
// bbx_pub_ptr_->Publish(frame_.objects_msg);