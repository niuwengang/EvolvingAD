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
    /*[5]--tools*/
    log_record_ptr_ = std::make_shared<LogRecord>(paramlist_.package_folder_path + "/log", "back_end");
}

bool BackEndPipe::Run()
{
    bool gnss_sync_flag = false;

    static Eigen::Matrix4f T_gnss2lidar = Eigen::Matrix4f::Identity();
    static bool T_gnss2lidar_init_flag = false;
    static int cnt = 50;
    static std::vector<Eigen::Vector3f> lidar_point_vec, gnss_point_vec;

    if (ReadMsg(gnss_sync_flag) == true)
    {
        if (gnss_sync_flag == true)
        {
            // log_record_ptr_->file_->info("timestamp:{}", frame_.time_stamp);
            gnss_odom_ptr_->InitPose(gnss_msg_);
            Eigen::Matrix4f gnss_pose = Eigen::Matrix4f::Identity();
            gnss_odom_ptr_->ComputePose(gnss_msg_, gnss_pose);

            if (T_gnss2lidar_init_flag == false)
            {
                if (cnt != 0)
                {
                    cnt--;
                    lidar_point_vec.push_back(Eigen::Vector3f(frame_.pose.block<3, 1>(0, 3)));
                    gnss_point_vec.push_back(Eigen::Vector3f(gnss_pose.block<3, 1>(0, 3)));
                }
                else
                {
                    OnlineCalibration(gnss_point_vec, lidar_point_vec, T_gnss2lidar);
                    T_gnss2lidar_init_flag = true;
                }
            }

            gnss_odom_pub_ptr_->Publish(gnss_pose);
        }
        if (T_gnss2lidar_init_flag == true)
        {
            frame_.pose = T_gnss2lidar * frame_.pose;
            lidar_odom_pub_ptr_->Publish(frame_.pose);
        }
    }

    return true;
}

// static std::deque<GnssMsg> unsynced_gnss_msg_queue;
// GnssMsg::TimeSync();

//     /*[1]--get the first message*/
//     GnssMsg gnss_msg = gnss_msg_queue_.front();
//     gnss_msg_queue_.pop_front();

//     /*[2]--check nan*/
//     if (std::isnan(gnss_msg.latitude + gnss_msg.longitude + gnss_msg.altitude))
//     {
//         return false;
//     }

//     /*[3]--init neu*/

//     /*[4]-calculate neu odom*/
//

// }

// const double gnss_lidar_time_diff =
//     gnss_msg.time_stamp -

// if (!gnss_msg_queue_.empty())
// {

//     log_record_ptr_->file_->info("gnss timestamp:\t{}", gnss_msg.time_stamp);

// }

//     log_record_ptr_->file_->info("lidar timestamp:\t{}", frame.time_stamp);

//     // CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
//     // pcl::transformPointCloud(*frame.cloud_msg.cloud_ptr, *transformed_cloud_ptr, frame.pose);

//     // cloud_pub_ptr_->Publish(transformed_cloud_ptr, 0.0);

//     // veh_tf_pub_ptr_->SendTransform(frame.pose);

//     // for (auto &object : frame.objects_msg.objects_vec)
//     // {
//     //     Eigen::Matrix4f object_pose;
//     //     object_pose.block<3, 1>(0, 3) << object.x, object.y, object.z;
//     //     object_pose.block<3, 3>(0, 0) = object.q.toRotationMatrix();

//     //     object_pose = frame.pose * object_pose;

//     //     object.x = object_pose(0, 3);
//     //     object.y = object_pose(1, 3);
//     //     object.z = object_pose(2, 3);
//     //     object.q = Eigen::Quaternionf(object_pose.block<3, 3>(0, 0));
//     // }

//     // bbx_pub_ptr_->Publish(frame.objects_msg);

bool BackEndPipe::ReadMsg(bool &gnss_sync_flag)
{
    gnss_sync_flag = false;

    gnss_sub_ptr_->ParseData(gnss_msg_queue_);

    if (frame_queue_.size() == 0)
    {
        return false;
    }

    frame_ = frame_queue_.front();
    frame_queue_.pop_front();

    const double refer_time = frame_.time_stamp;
    gnss_sync_flag = GnssMsg::TimeSync(gnss_msg_queue_, gnss_msg_, refer_time);

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

void BackEndPipe::OnlineCalibration(const std::vector<Eigen::Vector3f> &gnss_point_vec,
                                    const std::vector<Eigen::Vector3f> &lidar_point_vec, Eigen::Matrix4f &T_gnss2lidar)
{

    auto calculateAngle = [](double x1, double y1, double x2, double y2) -> float { // 计算点积
        double dotProduct = x1 * x2 + y1 * y2;

        double magnitudeA = std::sqrt(x1 * x1 + y1 * y1);
        double magnitudeB = std::sqrt(x2 * x2 + y2 * y2);

        double cosTheta = dotProduct / (magnitudeA * magnitudeB);

        double angleRadians = std::acos(cosTheta);

        double angleDegrees = angleRadians * (180.0 / M_PI);
        return angleDegrees;
    };

    float angleDegrees = calculateAngle(gnss_point_vec.back()(0), gnss_point_vec.back()(1), lidar_point_vec.back()(0),
                                        lidar_point_vec.back()(1));

    log_record_ptr_->file_->info("angleDegrees:{}", angleDegrees);

    CloudMsg::CLOUD_PTR source_cloud_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR target_cloud_ptr(new CloudMsg::CLOUD());

    for (int index = 0; index < gnss_point_vec.size(); index++)
    {
        CloudMsg::POINT point;
        point.x = gnss_point_vec[index](0);
        point.y = gnss_point_vec[index](1);
        point.z = 0;
        target_cloud_ptr->points.push_back(point);
    }

    for (int index = 0; index < lidar_point_vec.size(); index++)
    {
        CloudMsg::POINT point;
        point.x = lidar_point_vec[index](0);
        point.y = lidar_point_vec[index](1);
        point.z = 0; // lidar_point_vec[index](2);
        source_cloud_ptr->points.push_back(point);
    }

    pcl::IterativeClosestPoint<CloudMsg::POINT, CloudMsg::POINT> icp;
    icp.setInputSource(source_cloud_ptr);
    icp.setInputTarget(target_cloud_ptr);

    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);
    icp.setMaximumIterations(100);

    CloudMsg::CLOUD_PTR result_cloud_ptr(new CloudMsg::CLOUD());

    Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
    // if (abs(angleDegrees) > M_PI_2)
    // {
    //     Eigen::AngleAxisf rotationVector(M_PI, Eigen::Vector3f::UnitZ());
    //     Eigen::Matrix3f rotationMatrix = rotationVector.toRotationMatrix();
    //     initial_transform.block<3, 3>(0, 0) = rotationMatrix;
    //     icp.align(*result_cloud_ptr, initial_transform);
    // }
    // else
  //  {
        icp.align(*result_cloud_ptr);
    //}

    // std::cout << icp.getFinalTransformation() << std::endl;
    T_gnss2lidar = icp.getFinalTransformation();
}
} // namespace evolving_ad_ns