#include "imu_odom.hpp"

namespace evolving_ad_ns
{
ImuOdom::ImuOdom(const Eigen::Matrix4f &T_lidar2imu)
{
    this->T_lidar_imu_ = T_lidar2imu;
}
// ImuOdom::ImuOdom(const YAML::Node &config_node)
// {
// }

void ImuOdom::ComputeRelativePose(std::deque<ImuMsg> &imu_msg_queue, const double prev_time_stamp,
                                  const double curr_time_stamp, Eigen::Matrix4f &relative_pose)
{
    relative_pose = Eigen::Matrix4f::Identity();

    if (first_flag_ == false)
    {
        std::vector<ImuMsg> imu_msg_vec_selected;

        while (imu_msg_queue.empty() == false and (prev_time_stamp - imu_msg_queue.front().time_stamp) > 0.)
        {
            imu_msg_queue.pop_front();
        }

        for (const auto &it : imu_msg_queue)
        {
            double curr_time_stamp_dt = curr_time_stamp - it.time_stamp;
            double prev_time_stamp_dt = prev_time_stamp - it.time_stamp;

            if (curr_time_stamp_dt >= 0. and prev_time_stamp_dt <= 0.)
            {
                imu_msg_vec_selected.push_back(it);
            }
        }
        std::sort(imu_msg_vec_selected.begin(), imu_msg_vec_selected.end(),
                  [](const ImuMsg &imu_msg_1, const ImuMsg &imu_msg_2) {
                      return imu_msg_1.time_stamp < imu_msg_2.time_stamp;
                  });

        double curr_imu_time_stamp = 0., prev_imu_time_stamp = 0., dt = 0;

        Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
        for (uint32_t index = 0; index < imu_msg_vec_selected.size(); ++index)
        {
            Eigen::Vector3f w(imu_msg_vec_selected[index].angular_velocity.x,
                              imu_msg_vec_selected[index].angular_velocity.y,
                              imu_msg_vec_selected[index].angular_velocity.z);

            w = T_lidar_imu_.block<3, 3>(0, 0) * w;

            imu_msg_vec_selected[index].angular_velocity.x = w(0);
            imu_msg_vec_selected[index].angular_velocity.y = w(1);
            imu_msg_vec_selected[index].angular_velocity.z = w(2);

            if (prev_imu_time_stamp == 0.)
            {
                prev_imu_time_stamp = imu_msg_vec_selected[index].time_stamp;
                continue;
            }

            curr_imu_time_stamp = imu_msg_vec_selected[index].time_stamp;
            dt = curr_imu_time_stamp - prev_imu_time_stamp; // dt
            prev_imu_time_stamp = curr_imu_time_stamp;      // copy

            // Relative gyro propagation quaternion dynamics
            Eigen::Quaternionf qq = q;
            q.w() -= 0.5 * (qq.x() * w.x() + qq.y() * w.y() + qq.z() * w.z()) * dt;
            q.x() += 0.5 * (qq.w() * w.x() - qq.z() * w.y() + qq.y() * w.z()) * dt;
            q.y() += 0.5 * (qq.z() * w.x() + qq.w() * w.y() - qq.x() * w.z()) * dt;
            q.z() += 0.5 * (qq.x() * w.y() - qq.y() * w.x() + qq.w() * w.z()) * dt;
        }
        double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
        q.w() /= norm;
        q.x() /= norm;
        q.y() /= norm;
        q.z() /= norm;
        relative_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    }
    else
    {
        first_flag_ = false;
    }
}
} // namespace evolving_ad_ns
