/**
 * @file    imu_msg.cpp
 * @brief   imu消息封装
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "imu_msg.hpp"

bool ImuMsg::TimeSync(std::deque<ImuMsg> &unsynced_imu_msg_queue, std::deque<ImuMsg> &synced_imu_msg_queue,
                      const double sync_time)
{
    while (unsynced_imu_msg_queue.size() >= 2)
    {
        if (sync_time < unsynced_imu_msg_queue.at(0).time_stamp) // [0][1]超前于基准 退出等待
        {
            return false;
        }
        if (unsynced_imu_msg_queue.at(1).time_stamp < sync_time) //[0][1]落后于基准 弹出轮循
        {
            unsynced_imu_msg_queue.pop_front();
            continue;
        }
        else
        {
            ImuMsg t0_imu_msg = unsynced_imu_msg_queue.at(0);
            ImuMsg t1_imu_msg = unsynced_imu_msg_queue.at(1);
            ImuMsg synced_imu_msg;

            double coeff_t0 = (t1_imu_msg.time_stamp - sync_time) / (t1_imu_msg.time_stamp - t0_imu_msg.time_stamp);
            double coeff_t1 = (sync_time - t0_imu_msg.time_stamp) / (t1_imu_msg.time_stamp - t0_imu_msg.time_stamp);

            /*时间戳*/
            synced_imu_msg.time_stamp = sync_time;

            synced_imu_msg.linear_acceleration.x =
                t0_imu_msg.linear_acceleration.x * coeff_t0 + t1_imu_msg.linear_acceleration.x * coeff_t1;
            synced_imu_msg.linear_acceleration.y =
                t0_imu_msg.linear_acceleration.y * coeff_t0 + t1_imu_msg.linear_acceleration.y * coeff_t1;
            synced_imu_msg.linear_acceleration.z =
                t0_imu_msg.linear_acceleration.z * coeff_t0 + t1_imu_msg.linear_acceleration.z * coeff_t1;

            synced_imu_msg.angular_velocity.x =
                t0_imu_msg.angular_velocity.x * coeff_t0 + t1_imu_msg.angular_velocity.x * coeff_t1;
            synced_imu_msg.angular_velocity.y =
                t0_imu_msg.angular_velocity.y * coeff_t0 + t1_imu_msg.angular_velocity.y * coeff_t1;
            synced_imu_msg.angular_velocity.z =
                t0_imu_msg.angular_velocity.z * coeff_t0 + t1_imu_msg.angular_velocity.z * coeff_t1;

            Eigen::Quaterniond q0(t0_imu_msg.orientation.w, t0_imu_msg.orientation.x, t0_imu_msg.orientation.y,
                                  t0_imu_msg.orientation.z);
            Eigen::Quaterniond q1(t1_imu_msg.orientation.w, t1_imu_msg.orientation.x, t1_imu_msg.orientation.y,
                                  t1_imu_msg.orientation.z);

            Eigen::Quaterniond q = q0.slerp(coeff_t1, q1);
            synced_imu_msg.orientation.w = q.w();
            synced_imu_msg.orientation.x = q.x();
            synced_imu_msg.orientation.y = q.y();
            synced_imu_msg.orientation.z = q.z();

            synced_imu_msg_queue.push_back(synced_imu_msg);
            return true;
        }
    }

    return false;
}

Eigen::Matrix3f ImuMsg::GetOrientationMatrix()
{
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
}