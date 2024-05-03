#include "object_detect.hpp"

namespace evolving_ad_ns
{

/**
 * @brief get device info
 * @param[in] model_file_path
 * @return
 */
ObjectDetect::ObjectDetect(const std::string &model_file_path)
{
    /*[1]--get params*/
    model_file_path_ = model_file_path;

    /*[2]--get device info*/
    int count = 0;
    cudaGetDeviceCount(&count);
    spdlog::info("GPU num:{}", count);
    for (int index = 0; index < count; index++)
    {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, index);
        spdlog::info("GPU name:{}", prop.name);
        spdlog::info("GPU memory:{}MB", prop.totalGlobalMem >> 20); // byte-->kb-->mb 2^(10+10)
        spdlog::info("GPU capbility:{} {}", prop.major, prop.minor);
    }
    checkCudaErrors(cudaStreamCreate(&stream_));

    /*[3]--load infer model*/
    pointpillar_ptr_ = std::make_shared<PointPillar>(model_file_path_, stream_); // pointpillar ptr init

    /*[4]--some setting*/
    nms_pred_.reserve(100); // allocate memory
    setlocale(LC_ALL, "");  // env
}

/**
 * @brief 3D objecttion detection
 * @param[in] cloud_msg
 * @param[in] ods_msg
 * @return
 */
void ObjectDetect::Detect(const CloudMsg &cloud_msg, ObjectsMsg &objects_msg)
{
    /*[1]--reset variable*/
    nms_pred_.clear();               // clear nms predict
    objects_msg.objects_vec.clear(); // clear history

    /*[2]--pointpillar detection*/
    const size_t num_points = cloud_msg.cloud_ptr->points.size();
    const size_t num_features = 4; // x y z r
    float *points_in_cpu = new float[num_points * num_features];

    for (size_t index = 0; index < num_points; index++)
    {
        points_in_cpu[index * 4 + 0] = cloud_msg.cloud_ptr->points[index].x;
        points_in_cpu[index * 4 + 1] = cloud_msg.cloud_ptr->points[index].y;
        points_in_cpu[index * 4 + 2] = cloud_msg.cloud_ptr->points[index].z;
        points_in_cpu[index * 4 + 3] = 0.0f;
    }

    float *points_in_gpu = nullptr;                                                // destination data address
    unsigned int points_data_size = num_points * 4 * sizeof(float);                // destination data size
    checkCudaErrors(cudaMallocManaged((void **)&points_in_gpu, points_data_size)); // malloc memory for points_data
    checkCudaErrors(cudaMemcpy(points_in_gpu, points_in_cpu, points_data_size, cudaMemcpyDefault)); // copy a new one
    checkCudaErrors(cudaDeviceSynchronize());
    pointpillar_ptr_->doinfer(points_in_gpu, num_points, nms_pred_); // infer and out nms_pred_

    /*[3]--get result */
    objects_msg.time_stamp = cloud_msg.time_stamp;
    for (const auto box : nms_pred_)
    {
        ObjectMsg object_msg;

        object_msg.x = box.x;
        object_msg.y = box.y;
        object_msg.z = box.z;

        object_msg.w = box.w;
        object_msg.l = box.l;
        object_msg.h = box.h;

        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(box.rt, Eigen::Vector3f::UnitZ()));
        object_msg.q = yawAngle;

        object_msg.score = box.score;
        object_msg.id = box.id;

        if (object_msg.score > 0.6)
        {
            objects_msg.objects_vec.push_back(object_msg);
        }
    }

    /*[4]--free memory*/
    delete[] points_in_cpu;
    points_in_cpu = nullptr;
    cudaFree(points_in_gpu);
}

} // namespace evolving_ad_ns