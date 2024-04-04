/**
 * @file    odom_pipe.cpp
 * @brief   里程计pipeline
 * @author  niu_wengang@163.com
 * @date    2024-03-27
 * @version 0.1.0
 */

// related header
#include "odom_pipe.hpp"

/**
 * @brief OdomPipe有参构造
 * @param[in] nh  ros句柄
 * @return
 */
OdomPipe::OdomPipe(ros::NodeHandle &nh)
{
    /*1--加载参数*/
    paramlist_ptr_ = std::make_shared<Tools::ParamList>();
    paramlist_ptr_->package_folder_path = ros::package::getPath("urban_nav");
    /*1.1--工程路径*/
    YAML::Node config = YAML::LoadFile(paramlist_ptr_->package_folder_path + "/config/param.yaml");
    paramlist_ptr_->result_folder_path = paramlist_ptr_->package_folder_path + "/result";
    paramlist_ptr_->keyframe_folder_path = paramlist_ptr_->package_folder_path + "/result/keyframe";
    /*1.2--话题消息*/
    paramlist_ptr_->imu_sub_topic = config["topic_sub"]["imu_sub_topic"].as<std::string>();
    paramlist_ptr_->gnss_sub_topic = config["topic_sub"]["gnss_sub_topic"].as<std::string>();
    paramlist_ptr_->cloud_sub_topic = config["topic_sub"]["cloud_sub_topic"].as<std::string>();
    /*1.3--关键帧参数*/
    paramlist_ptr_->keyframe_distance = config["local_map"]["keyframe_distance"].as<float>();
    paramlist_ptr_->keyframe_num = config["local_map"]["keyframe_num"].as<unsigned int>();
    /*1.4--标定参数*/
    paramlist_ptr_->lidar_to_body = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
        config["extrinsic"]["lidar_to_body"].as<std::vector<float>>().data());
    paramlist_ptr_->imu_to_body = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
        config["extrinsic"]["imu_to_body"].as<std::vector<float>>().data());
    /*1.5--滤波参数*/
    paramlist_ptr_->single_scan_filter_size = config["voxel_filter_size"]["single_scan"].as<float>();
    paramlist_ptr_->local_map_filter_size = config["voxel_filter_size"]["local_map"].as<float>();
    paramlist_ptr_->global_map_filter_size = config["voxel_filter_size"]["global_map"].as<float>();
    /*1.6--输出设定*/
    paramlist_ptr_->save_global_map = config["output_setting"]["save_global_map"].as<bool>();

    /*2--工具设置*/
    /*2.1--系统监测*/
    log_ptr_ = std::make_shared<Tools::LogRecord>(paramlist_ptr_->package_folder_path + "/log", "odom_node");
    time_ptr_ = std::make_shared<Tools::TimeRecord>();
    /*2.2--imu收发*/
    imu_sub_ptr_ = std::make_shared<Tools::ImuSub>(nh, paramlist_ptr_->imu_sub_topic);
    imu_pub_ptr_ = std::make_shared<Tools::ImuPub>(nh, "urban_imu", "map");
    /*2.3--点云收发*/
    cloud_sub_ptr_ = std::make_shared<Tools::CloudSub>(nh, paramlist_ptr_->cloud_sub_topic);
    cloud_pub_ptr_ = std::make_shared<Tools::CloudPub>(nh, "urban_cloud", "map");
    /*2.4--gnss收发*/
    gnss_sub_ptr_ = std::make_shared<Tools::GnssSub>(nh, paramlist_ptr_->gnss_sub_topic);
    gnss_pub_ptr_ = std::make_shared<Tools::GnssPub>(nh, "urban_gnss_init", "map");
    /*2.5--里程计发布*/
    body_odom_pub_ptr_ = std::make_shared<Tools::OdomPub>(nh, "urban_body_odom", "map", "body");
    gnss_odom_pub_ptr_ = std::make_shared<Tools::OdomPub>(nh, "urban_gnss_odom", "map", "gnss");
    lidar_odom_pub_ptr_ = std::make_shared<Tools::OdomPub>(nh, "urban_lidar_odom", "map", "lidar");
    /*2.6--tf树发布*/
    body_tf_pub_ptr_ = std::make_shared<Tools::TfPub>("map", "ground_link"); // tf树

    /*3--算法模块*/
    single_scan_filter_ptr_ = std::make_shared<Module::VoxelFilter>(paramlist_ptr_->single_scan_filter_size);
    local_map_filter_ptr_ = std::make_shared<Module::VoxelFilter>(paramlist_ptr_->local_map_filter_size);
    global_map_filter_ptr_ = std::make_shared<Module::VoxelFilter>(paramlist_ptr_->global_map_filter_size);

    registration_ptr_ = std::make_shared<Module::FastGicpRegistration>(1.0, 0.1, 0.01, 30);

    /*4--变量初始化*/
    single_scan_ptr_.reset(new CloudMsg::CLOUD());
    local_map_ptr_.reset(new CloudMsg::CLOUD());
    global_map_ptr_.reset(new CloudMsg::CLOUD());

    /*5--文件夹建立*/
    Tools::FileManager::CreateFolder(paramlist_ptr_->result_folder_path);
}

/**
 * @brief OdomPipe运行
 * @param[in] none
 * @return
 */
void OdomPipe::Run()
{
    /*1--读取缓冲区*/
    if (ReadBuffer() == false)
    {
        return;
    }
    /*2--初始化gnss里程计*/
    if (InitGnssOdom() == false)
    {
        return;
    }
    /*3--读取当前消息*/
    while (ReadMsg() == true)
    {
        // log_ptr_->file_->info("imu时间戳:{}", cur_imu_msg_.time_stamp);
        // log_ptr_->file_->info("gnss时间戳:{}", cur_gnss_msg_.time_stamp);

        /*3.1--更新gnss里程计*/
        UpdateGnssOdom(gnss_odom_);
        gnss_odom_pub_ptr_->Publish(gnss_odom_);

        /*3.2--更新雷达里程计*/
        InitLidarOdom();
        /// time_ptr_->Start();
        UpdateLidarOdom(cur_cloud_msg_, lidar_odom_);
        // log_ptr_->terminal_->info("频率:{}", time_ptr_->End());
        // log_ptr_->terminal_->info("cloud队列数据:{}", cloud_msg_queue_.size());

        /*3.3--可视化*/
        lidar_odom_pub_ptr_->Publish(lidar_odom_);
        body_tf_pub_ptr_->SendTransform(lidar_odom_);
        cloud_pub_ptr_->Publish(local_map_ptr_);

        /*3.4--文件保存*/
        SaveTrajectory(lidar_odom_, gnss_odom_);
    }
}

/**
 * @brief 读取缓冲区
 * @param[in]
 * @return
 */
bool OdomPipe::ReadBuffer()
{

    static std::deque<ImuMsg> unsynced_imu_msg_queue;
    static std::deque<GnssMsg> unsynced_gnss_msg_queue;

    /*1--数据读取*/
    cloud_sub_ptr_->ParseData(cloud_msg_queue_);
    imu_sub_ptr_->ParseData(unsynced_imu_msg_queue);
    gnss_sub_ptr_->ParseData(unsynced_gnss_msg_queue);

    if (cloud_msg_queue_.size() == 0)
    {
        return false;
    }
    /*2--缓冲区同步*/
    const double refer_time = cloud_msg_queue_.front().time_stamp;

    bool valid_imu_flag = ImuMsg::TimeSync(unsynced_imu_msg_queue, imu_msg_queue_, refer_time);
    bool valid_gnss_flag = GnssMsg::TimeSync(unsynced_gnss_msg_queue, gnss_msg_queue_, refer_time);

    /*3--同步初始化*/
    static bool sensor_inited = false;
    if (!sensor_inited)
    {
        if (!valid_imu_flag or !valid_gnss_flag)
        {
            cloud_msg_queue_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

/**
 * @brief 读取当前消息
 * @param[in]
 * @return
 */
bool OdomPipe::ReadMsg()
{
    if (imu_msg_queue_.empty() or cloud_msg_queue_.empty() or gnss_msg_queue_.empty())
    {
        return false;
    }

    cur_cloud_msg_ = cloud_msg_queue_.front();
    cur_imu_msg_ = imu_msg_queue_.front();
    cur_gnss_msg_ = gnss_msg_queue_.front();

    const double refer_time =
        std::max(std::max(cur_cloud_msg_.time_stamp, cur_gnss_msg_.time_stamp), cur_imu_msg_.time_stamp);

    const double cloud_refer_timediff = fabs(refer_time - cur_cloud_msg_.time_stamp);
    const double imu_refer_timediff = fabs(refer_time - cur_imu_msg_.time_stamp);
    const double gnss_refer_timediff = fabs(refer_time - cur_gnss_msg_.time_stamp);

    if (cloud_refer_timediff < 0.05 and imu_refer_timediff < 0.05 and gnss_refer_timediff < 0.05)
    {
        cloud_msg_queue_.pop_front();
        imu_msg_queue_.pop_front();
        gnss_msg_queue_.pop_front();
        return true;
    }
    else
    {
        if (cloud_refer_timediff > 0.05) // cloud时间落后
        {
            cloud_msg_queue_.pop_front();
        }
        if (imu_refer_timediff > 0.05) // imu时间落后
        {
            imu_msg_queue_.pop_front();
        }
        if (gnss_refer_timediff > 0.05) // gnss时间落后
        {
            gnss_msg_queue_.pop_front();
        }
        return false;
    }
}

/**
 * @brief 初始化gnss里程计
 * @param[in]
 * @return
 */
bool OdomPipe::InitGnssOdom()
{
    static bool gnss_init = false;
    if (!gnss_init and !gnss_msg_queue_.empty())
    {
        GnssMsg gnss_msg = gnss_msg_queue_.front();
        gnss_msg.PosInit();
        gnss_init = true;
    }
    return gnss_init;
}

/**
 * @brief gnss里程计更新
 * @param[in out] gnss_odom
 * @return
 */
bool OdomPipe::UpdateGnssOdom(Eigen::Matrix4f &gnss_odom)
{
    gnss_odom.block<3, 1>(0, 3) = cur_gnss_msg_.OdomUpdate();
    gnss_odom.block<3, 3>(0, 0) = cur_imu_msg_.GetOrientationMatrix();
    gnss_odom = paramlist_ptr_->lidar_to_body.inverse() * gnss_odom; // 对齐至雷达坐标系
    return true;
}

/**
 * @brief 初始化雷达里程计
 * @param[in]
 * @return
 */
bool OdomPipe::InitLidarOdom()
{
    static bool lidar_init = false;
    if (lidar_init == false)
    {
        lidar_odom_init_pose_ = gnss_odom_;
        lidar_init = true;
    }
    return lidar_init;
}

/**
 * @brief 雷达里程计更新
 * @param[in] cloud_msg
 * @param[in out] lidar_odom 雷达里程计
 * @return
 */
bool OdomPipe::UpdateLidarOdom(const CloudMsg &cloud_msg, Eigen::Matrix4f &lidar_odom)
{

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = lidar_odom_init_pose_;
    static Eigen::Matrix4f predict_pose = lidar_odom_init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = lidar_odom_init_pose_;

    /*1--创建当前帧*/
    Frame current_frame;
    current_frame.cloud_msg.time_stamp = cloud_msg.time_stamp;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_msg.cloud_ptr, *current_frame.cloud_msg.cloud_ptr, indices);

    /*2--第一帧*/
    if (local_map_frames_.empty() == true)
    {
        lidar_odom = current_frame.pose = lidar_odom_init_pose_; // 初始位姿势 若存在imu可设定方向
        UpdateLocalMap(current_frame);
        return true;
    }
    /*3--非第一帧*/
    /*3.1--点云配准*/
    CloudMsg::CLOUD_PTR registered_cloud_ptr(new CloudMsg::CLOUD());

    CloudMsg::CLOUD_PTR filtered_current_scan_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR filtered_local_map_ptr(new CloudMsg::CLOUD());

    single_scan_filter_ptr_->Filter(current_frame.cloud_msg.cloud_ptr, filtered_current_scan_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);

    registration_ptr_->SetSourceCloud(filtered_current_scan_ptr);
    registration_ptr_->SetTargetCloud(filtered_local_map_ptr);

    registration_ptr_->Registration(predict_pose, current_frame.pose, registered_cloud_ptr);
    lidar_odom = current_frame.pose;

    /*3.2--运动预测*/
    step_pose = last_pose.inverse() * current_frame.pose; // 右乘 相对于当前系统
    predict_pose = current_frame.pose * step_pose;
    last_pose = current_frame.pose;
    /*3.3--关键帧检测*/
    if (fabs(last_key_frame_pose(0, 3) - current_frame.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame.pose(2, 3)) >
        paramlist_ptr_->keyframe_distance)
    {
        UpdateLocalMap(current_frame);
        last_key_frame_pose = current_frame.pose;
    }

    return true;
}

/**
 * @brief 更新局部地图
 * @param[in] new_key_frame
 * @return
 */
bool OdomPipe::UpdateLocalMap(const Frame &new_key_frame)
{
    /*1--新关键帧入队*/
    Frame key_frame = new_key_frame;
    key_frame.cloud_msg.cloud_ptr.reset(new CloudMsg::CLOUD(*new_key_frame.cloud_msg.cloud_ptr)); // 深拷贝
    local_map_frames_.push_back(key_frame);

    /*2--维护关键帧队列*/
    while (local_map_frames_.size() > static_cast<size_t>(paramlist_ptr_->keyframe_num))
    {
        local_map_frames_.pop_front();
    }

    /*3--拼接localmap*/
    std::vector<CloudMsg::CLOUD> cloud_vector(local_map_frames_.size());
#pragma omp parallel for
    for (unsigned int i = 0; i < local_map_frames_.size(); i++)
    {
        CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_msg.cloud_ptr, *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        cloud_vector[i] = *transformed_cloud_ptr;
    }
    local_map_ptr_.reset(new CloudMsg::CLOUD());
    for (unsigned int i = 0; i < cloud_vector.size(); i++)
    {
        *local_map_ptr_ += cloud_vector[i];
    }
    return true;
}

/**
 * @brief 轨迹保存
 * @param[in] new_key_frame
 * @return
 */
void OdomPipe::SaveTrajectory(const Eigen::Matrix4f &lidar_odom, const Eigen::Matrix4f &gnss_odom)
{
    static std::ofstream ground_truth_stream, laser_odom_stream;
    static bool is_file_created = false;
    /*1--创建文件*/
    if (!is_file_created)
    {
        if (Tools::FileManager::CreateTxtFile(ground_truth_stream,
                                              paramlist_ptr_->result_folder_path + "/gnss_odom.txt"))
        {
            log_ptr_->file_->info("gnss odom已创建");
        }
        else
        {
            log_ptr_->file_->error("gnss odom创建失败");
            exit(EXIT_FAILURE);
        }

        if (Tools::FileManager::CreateTxtFile(laser_odom_stream,
                                              paramlist_ptr_->result_folder_path + "/lidar_odom.txt"))
        {
            log_ptr_->file_->info("lidar odom已创建");
        }
        else
        {
            log_ptr_->file_->error("lidar odom创建失败");
            exit(EXIT_FAILURE);
        }
        is_file_created = true;
    }
    /*2--写入轨迹*/
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            ground_truth_stream << gnss_odom(i, j);
            laser_odom_stream << lidar_odom(i, j);
            if (i == 2 && j == 3)
            {
                ground_truth_stream << std::endl;
                laser_odom_stream << std::endl;
            }
            else
            {
                ground_truth_stream << " ";
                laser_odom_stream << " ";
            }
        }
    }
}