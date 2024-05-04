#include "frame.hpp"

namespace evolving_ad_ns
{

// Frame::Frame(const CloudMsg &in_cloud_msg)
// {
//     time_stamp = in_cloud_msg.time_stamp; // extract cloud time stamp
//     *this->cloud_msg.cloud_ptr = *in_cloud_msg.cloud_ptr;
// }

Frame &Frame::operator=(const Frame &other)
{
    this->time_stamp = other.time_stamp;                           // timestamp
    this->pose = other.pose;                                       // pose
    *(this->cloud_msg.cloud_ptr) = *(other.cloud_msg.cloud_ptr);   // cloud
    this->objects_msg.objects_vec = other.objects_msg.objects_vec; // ods_vec
    return *this;
}

Frame::Frame(const Frame &other)
{
    this->time_stamp = other.time_stamp;                           // timestamp
    this->pose = other.pose;                                       // pose
    *(this->cloud_msg.cloud_ptr) = *(other.cloud_msg.cloud_ptr);   // cloud
    this->objects_msg.objects_vec = other.objects_msg.objects_vec; // ods_vec
}

} // namespace evolving_ad_ns