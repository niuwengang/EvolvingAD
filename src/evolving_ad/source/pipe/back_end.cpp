#include "back_end.hpp"

namespace evolving_ad_ns
{
BackEndPipe::BackEndPipe(ros::NodeHandle &nh, const std::string package_folder_path)
{
}
bool BackEndPipe::Run()
{
    return true;
}

void BackEndPipe::ReveiveFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex)
{
    mutex.lock();
    if (!frame_queue.empty())
    {
        for (int i = 0; i < frame_queue.size(); i++)
        {
            Frame frame; // deep copy
            frame.index = frame_queue.at(i).index;
            frame.time_stamp = frame_queue.at(i).time_stamp;
            frame.pose = frame_queue.at(i).pose;
            *frame.cloud_msg.cloud_ptr = *frame_queue.at(i).cloud_msg.cloud_ptr;

            frame_queue_.push_back(frame);
            std::cout << "queue size:" << frame_queue_.size() << std::endl;
        }
        frame_queue.clear();
    }
    mutex.unlock();
}
} // namespace evolving_ad_ns