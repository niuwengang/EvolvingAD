#include "object_track.hpp"

namespace evolving_ad_ns
{

ObjectTrack::ObjectTrack()
{
}

ObjectTrack::~ObjectTrack()
{
}

void ObjectTrack::Track(ObjectsMsg &objects_msg_curr, const Eigen::Matrix4f &relative_pose)
{

    if (first_flag_ == false)
    {
        /*1.align cooridinates to now*/

        /*2--kalman predict*/
        for (auto &object : objects_msg_esti_.objects_vec)
        {
            object.KfPredict();
        }
        objects_msg_esti_.TransCoord(relative_pose); // align to current object
        /*2.data association*/
        const unsigned int curr_size = objects_msg_curr.objects_vec.size();  // row
        const unsigned int esti_size = objects_msg_esti_.objects_vec.size(); // col
        if (curr_size != 0 and esti_size != 0)
        {
            std::vector<std::vector<double>> costMatrix(curr_size, std::vector<double>(esti_size, 0.0));
            for (unsigned int index_curr = 0; index_curr < curr_size; index_curr++) // row
            {
                for (unsigned int index_esti = 0; index_esti < esti_size; index_esti++) // col
                {
                    costMatrix[index_curr][index_esti] =
                        pow(objects_msg_esti_.objects_vec[index_esti].x - objects_msg_curr.objects_vec[index_curr].x,
                            2) +
                        pow(objects_msg_esti_.objects_vec[index_esti].y - objects_msg_curr.objects_vec[index_curr].y,
                            2) +
                        pow(objects_msg_esti_.objects_vec[index_esti].z - objects_msg_curr.objects_vec[index_curr].z,
                            2);
                }
            }

            HungarianAlgorithm HungAlgo;
            std::vector<int> assignment;
            double cost = HungAlgo.Solve(costMatrix, assignment);
            for (int index = 0; index < costMatrix.size(); index++)
            {
                int index_match = assignment[index];
                // d_match&&pmatch
                if (index_match >= 0) // todo:cost filter
                {
                    if (costMatrix[index][index_match] <= 10) // todo 3D IoU and dis
                    {
                        objects_msg_curr.objects_vec[index].id = objects_msg_esti_.objects_vec[index_match].id;
                        objects_msg_esti_.objects_vec[index_match].KfUpdate(objects_msg_curr.objects_vec[index]);
                    }
                }
                else // d_unmatch add
                {
                    objects_msg_curr.objects_vec[index].id = ++id_;
                    objects_msg_esti_.objects_vec.push_back(objects_msg_curr.objects_vec[index]);
                }
            }
        }
        else if (curr_size != 0) //
        {
            for (unsigned int index = 0; index < objects_msg_curr.objects_vec.size(); index++)
            {
                objects_msg_curr.objects_vec[index].id = ++id_;
                objects_msg_esti_.objects_vec.push_back(objects_msg_curr.objects_vec[index]);
            }
        }
        else
        {
        }

        objects_msg_esti_.objects_vec.erase(std::remove_if(objects_msg_esti_.objects_vec.begin(),
                                                           objects_msg_esti_.objects_vec.end(),
                                                           [](const ObjectMsg &obj) { return obj.lifetime <= 0.6; }),
                                            objects_msg_esti_.objects_vec.end());

        objects_msg_curr.objects_vec.clear();
        objects_msg_curr = objects_msg_esti_;
    }
    else // init
    {
        first_flag_ = false;
        id_ = 1;
        for (auto &object : objects_msg_curr.objects_vec)
        {
            object.id = id_++;
        }
        objects_msg_esti_ = objects_msg_curr;
    }
}
} // namespace evolving_ad_ns
