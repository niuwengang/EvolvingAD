#include "frame.hpp"

namespace evolving_ad_ns
{

NormalFrame::NormalFrame(const CloudMsg &in_cloud_msg)
{
    time_stamp = in_cloud_msg.time_stamp; // extract cloud time stamp
    *this->cloud_msg.cloud_ptr = *in_cloud_msg.cloud_ptr;
}

} // namespace evolving_ad_ns