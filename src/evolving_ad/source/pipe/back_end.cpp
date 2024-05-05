#include "back_end.hpp"

namespace evolving_ad_ns
{
BackEndPipe::BackEndPipe(ros::NodeHandle &nh, const std::string package_folder_path)
{
    /*[1]--load params*/
    paramlist_.package_folder_path = package_folder_path;
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/ad.yaml");

    paramlist_.gnss_sub_topic = config_node["topic_sub"]["gnss_sub_topic"].as<std::string>();
    paramlist_.gt_sub_topic = config_node["topic_sub"]["gt_sub_topic"].as<std::string>();
    /*[2]--sub*/
    gnss_sub_ptr_ = std::make_shared<GnssSub>(nh, paramlist_.gnss_sub_topic);
    gt_sub_ptr_ = std::make_shared<GtSub>(nh, paramlist_.gt_sub_topic);
    /*[3]--pub*/
    gnss_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_gnss_odom", "map", "gnss");
    gt_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_gnss_gt", "map", "gnss");
    cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "back_end_cloud", "map");
    veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "back_end_bbx", "map");
    lidar_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "back_end_lidar_odom", "map", "lidar");
    /*[4]--algorithm module*/
    gnss_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
    gt_odom_ptr_ = std::make_shared<GnssOdom>(config_node["lidar_odom"]);
    /*[5]--tools*/
    log_record_ptr_ = std::make_shared<LogRecord>(paramlist_.package_folder_path + "/log", "back_end");
}

bool BackEndPipe::Run()
{

    gt_sub_ptr_->ParseData(gt_msg_queue_);
    if (gt_msg_queue_.size() > 0)
    {
        gt_msg_ = gt_msg_queue_.front();
        gt_msg_queue_.pop_front();

        gt_odom_ptr_->InitPose(gt_msg_);
        Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
        gnss_odom_ptr_->ComputePose(gt_msg_, gt_pose);

        gt_odom_pub_ptr_->Publish(gt_pose);
    }

    if (frame_queue_.size() > 0)
    {
        frame_ = frame_queue_.front();
        frame_queue_.pop_front();
    }

// #define NOT_READY
#ifdef NOT_READY

    bool gnss_sync_flag = false;

    static Eigen::Matrix4f T_gnss2lidar = Eigen::Matrix4f::Identity();

    static bool T_gnss2lidar_init_flag = false;
    static int cnt = 10;                                                 // temp
    static std::vector<Eigen::Vector3f> lidar_point_vec, gnss_point_vec; // temp

    if (ReadMsg(gnss_sync_flag) == true)
    {
        if (gnss_sync_flag == true)
        {
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
            /*[1]--lidar odom display*/
            frame_.pose = T_gnss2lidar * frame_.pose;
            lidar_odom_pub_ptr_->Publish(frame_.pose);
            veh_tf_pub_ptr_->SendTransform(frame_.pose);

            /*[2]--cloud  display*/
            CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
            pcl::transformPointCloud(*frame_.cloud_msg.cloud_ptr, *transformed_cloud_ptr, frame_.pose);
            cloud_pub_ptr_->Publish(transformed_cloud_ptr, 0.0);

            /*[3]--object display*/
            for (auto &object : frame_.objects_msg.objects_vec)
            {
                Eigen::Matrix4f object_pose;
                object_pose.block<3, 1>(0, 3) << object.x, object.y, object.z;
                object_pose.block<3, 3>(0, 0) = object.q.toRotationMatrix();

                object_pose = frame_.pose * object_pose;

                object.x = object_pose(0, 3);
                object.y = object_pose(1, 3);
                object.z = object_pose(2, 3);
                object.q = Eigen::Quaternionf(object_pose.block<3, 3>(0, 0));
            }
            bbx_pub_ptr_->Publish(frame_.objects_msg);
        }
    }

#endif
    return true;
}

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

    auto calculateAngle = [](double x1, double y1, double x2, double y2) -> float {
        double dotProduct = x1 * x2 + y1 * y2;

        double magnitudeA = std::sqrt(x1 * x1 + y1 * y1);
        double magnitudeB = std::sqrt(x2 * x2 + y2 * y2);

        double cosTheta = dotProduct / (magnitudeA * magnitudeB);

        double angleRadians = std::acos(cosTheta);

        return angleRadians;
    };

    float angleDegrees = calculateAngle(gnss_point_vec.back()(0), gnss_point_vec.back()(1), lidar_point_vec.back()(0),
                                        lidar_point_vec.back()(1));

    log_record_ptr_->file_->info("angleDegrees:{}", angleDegrees);

    CloudMsg::CLOUD_PTR source_cloud_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR target_cloud_ptr(new CloudMsg::CLOUD());

    for (size_t index = 0; index < gnss_point_vec.size(); index++)
    {
        CloudMsg::POINT point;
        point.x = gnss_point_vec[index](0);
        point.y = gnss_point_vec[index](1);
        point.z = 0;
        target_cloud_ptr->points.push_back(point);
    }

    for (size_t index = 0; index < lidar_point_vec.size(); index++)
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

    if (abs(angleDegrees) > M_PI_2)
    {
        Eigen::AngleAxisf rotationVector(M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotationMatrix = rotationVector.toRotationMatrix();
        initial_transform.block<3, 3>(0, 0) = rotationMatrix;
        icp.align(*result_cloud_ptr, initial_transform);
    }
    else
    {
        icp.align(*result_cloud_ptr);
    }

    // std::cout << icp.getFinalTransformation() << std::endl;
    T_gnss2lidar = icp.getFinalTransformation();
}
} // namespace evolving_ad_ns