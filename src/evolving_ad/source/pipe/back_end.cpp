#include "back_end.hpp"

namespace evolving_ad_ns
{
BackEndPipe::BackEndPipe(ros::NodeHandle &nh, const std::string package_folder_path)
{
    /*[1]--load params*/
    paramlist_.package_folder_path = package_folder_path;
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/ad.yaml");

    paramlist_.gnss_sub_topic = config_node["topic_sub"]["gnss_sub_topic"].as<std::string>();
    /*[2]--sub*/
    gnss_sub_ptr_ = std::make_shared<GnssSub>(nh, paramlist_.gnss_sub_topic);
    /*[3]--pub*/
    gnss_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_gnss_odom", "map", "gnss");
    cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "back_end_cloud", "map");
    veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "back_end_bbx", "map");
    lidar_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_lidar_odom", "map", "lidar");
    /*[4]--algorithm module*/
    gnss_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
}

bool BackEndPipe::Run()
{

    gnss_sub_ptr_->ParseData(gnss_msg_queue_);
    if (!gnss_msg_queue_.empty())
    {
        GnssMsg gnss_msg = gnss_msg_queue_.front();
        gnss_msg_queue_.pop_front();

        gnss_odom_ptr_->InitPose(gnss_msg);
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        gnss_odom_ptr_->ComputePose(gnss_msg, pose);
        gnss_odom_pub_ptr_->Publish(pose, 0);
    }

    if (!frame_queue_.empty())
    {
        Frame frame = frame_queue_.front();
        frame_queue_.pop_front();

        CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
        pcl::transformPointCloud(*frame.cloud_msg.cloud_ptr, *transformed_cloud_ptr, frame.pose);

        cloud_pub_ptr_->Publish(transformed_cloud_ptr, 0.0);
        lidar_odom_pub_ptr_->Publish(frame.pose, 0);
        veh_tf_pub_ptr_->SendTransform(frame.pose);

        std::cout << "bbx queue:" << frame.objects_msg.objects_vec.size() << std::endl;
        for (auto &object : frame.objects_msg.objects_vec)
        {
            Eigen::Matrix4f object_pose;
            object_pose.block<3, 1>(0, 3) << object.x, object.y, object.z;
            object_pose.block<3, 3>(0, 0) = object.q.toRotationMatrix();

            object_pose = frame.pose * object_pose;

            object.x = object_pose(0, 3);
            object.y = object_pose(1, 3);
            object.z = object_pose(2, 3);

            object.q = Eigen::Quaternionf(object_pose.block<3, 3>(0, 0));
        }

        bbx_pub_ptr_->Publish(frame.objects_msg);
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
            Frame frame; // deep copy
            frame.index = frame_queue.at(i).index;
            frame.time_stamp = frame_queue.at(i).time_stamp;
            frame.pose = frame_queue.at(i).pose;
            *frame.cloud_msg.cloud_ptr = *frame_queue.at(i).cloud_msg.cloud_ptr;
            frame.objects_msg.objects_vec = frame_queue.at(i).objects_msg.objects_vec;

            frame_queue_.push_back(frame);
        }
        frame_queue.clear();
    }
    mutex.unlock();
}
} // namespace evolving_ad_ns