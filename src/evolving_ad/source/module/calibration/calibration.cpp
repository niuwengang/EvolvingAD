#include "calibration.hpp"

namespace evolving_ad_ns
{
Lidar2GnssCalibration::Lidar2GnssCalibration(const float distance_step, const unsigned int max_sampling_num)
{
    distance_step_ = distance_step;
    max_sampling_num_ = max_sampling_num;

    gnss_point_vec_.push_back(Eigen::Vector3f::Zero());
    lidar_point_vec_.push_back(Eigen::Vector3f::Zero());

    accumulate_gnss_odom_ = 0;
    accumulate_lidar_odom_ = 0;
}

bool Lidar2GnssCalibration::Calibration(const Eigen::Vector3f gnss_point, const Eigen::Vector3f lidar_point,
                                        Eigen::Matrix4f &T_gnss2lidar)
{
    double dis1 =
        sqrt(pow(gnss_point_vec_.back()(0) - gnss_point(0), 2) + pow(gnss_point_vec_.back()(1) - gnss_point(1), 2));

    if (dis1 >= distance_step_)
    {
        gnss_point_vec_.push_back(gnss_point);
    }

    double dis2 =
        sqrt(pow(lidar_point_vec_.back()(0) - lidar_point(0), 2) + pow(lidar_point_vec_.back()(1) - lidar_point(1), 2));

    if (dis2 >= distance_step_)
    {
        lidar_point_vec_.push_back(lidar_point);
    }

    auto calculateAngleBetweenVectors = [](double target_x, double target_y, double source_x,
                                           double source_y) -> float {
        // double dotProduct = x1 * x2 + y1 * y2;

        // double magnitudeA = std::sqrt(x1 * x1 + y1 * y1);
        // double magnitudeB = std::sqrt(x2 * x2 + y2 * y2);

        // double cosTheta = dotProduct / (magnitudeA * magnitudeB);

        // double angleRadians = std::acos(cosTheta);

        // return angleRadians;
        float diff = atan2(target_y, target_x) - atan2(source_y, source_x);

        if (diff < 0)
        {
            diff += 2 * M_PI;
        }
        return diff;
    };

    if (gnss_point_vec_.size() >= max_sampling_num_ and lidar_point_vec_.size() >= max_sampling_num_)
    {
        CloudMsg::CLOUD_PTR source_cloud_ptr(new CloudMsg::CLOUD());
        CloudMsg::CLOUD_PTR target_cloud_ptr(new CloudMsg::CLOUD());

        for (size_t index = 0; index < gnss_point_vec_.size(); index++)
        {
            CloudMsg::POINT point;
            point.x = gnss_point_vec_[index](0);
            point.y = gnss_point_vec_[index](1);
            point.z = 0;
            target_cloud_ptr->points.push_back(point);
        }

        for (size_t index = 0; index < lidar_point_vec_.size(); index++)
        {
            CloudMsg::POINT point;
            point.x = lidar_point_vec_[index](0);
            point.y = lidar_point_vec_[index](1);
            point.z = 0; // lidar_point_vec[index](2);
            source_cloud_ptr->points.push_back(point);
        }

        pcl::IterativeClosestPoint<CloudMsg::POINT, CloudMsg::POINT> icp;
        icp.setInputSource(source_cloud_ptr);
        icp.setInputTarget(target_cloud_ptr);

        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(0.5);
        icp.setMaximumIterations(100);

        float yaw_inital = calculateAngleBetweenVectors(gnss_point_vec_.back()(0), gnss_point_vec_.back()(1),
                                                        lidar_point_vec_.back()(0), lidar_point_vec_.back()(1));

        CloudMsg::CLOUD_PTR result_cloud_ptr(new CloudMsg::CLOUD());
        // if (fabs(yaw_inital) > M_PI_2)
        //{
        Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
        Eigen::AngleAxisf rotationVector(yaw_inital, Eigen::Vector3f::UnitZ()); // anticlockwise
        initial_transform.block<3, 3>(0, 0) = rotationVector.toRotationMatrix();
        icp.align(*result_cloud_ptr, initial_transform);
        //  }
        // else
        // {
        //     icp.align(*result_cloud_ptr);
        // }
        T_gnss2lidar = icp.getFinalTransformation();
        return true;
    }
    else
    {
        return false;
    }
}

} // namespace evolving_ad_ns

// auto calculateAngle = [](double x1, double y1, double x2, double y2) -> float {
//     double dotProduct = x1 * x2 + y1 * y2;

//     double magnitudeA = std::sqrt(x1 * x1 + y1 * y1);
//     double magnitudeB = std::sqrt(x2 * x2 + y2 * y2);

//     double cosTheta = dotProduct / (magnitudeA * magnitudeB);

//     double angleRadians = std::acos(cosTheta);

//     return angleRadians;
// };

// float angleDegrees = calculateAngle(gnss_point_vec.back()(0), gnss_point_vec.back()(1), lidar_point_vec.back()(0),
//                                     lidar_point_vec.back()(1));

// log_record_ptr_->file_->info("angleDegrees:{}", angleDegrees);
