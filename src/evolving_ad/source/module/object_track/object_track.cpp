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
        objects_msg_prev_.TransCoord(relative_pose); // align to current object

        const unsigned int curr_size = objects_msg_curr.objects_vec.size();  // row
        const unsigned int prev_size = objects_msg_prev_.objects_vec.size(); // col
        if (curr_size == 0 or prev_size == 0)
        {
            return;
        }

        std::vector<std::vector<double>> costMatrix(curr_size, std::vector<double>(prev_size, 0.0));
        for (unsigned int index_curr = 0; index_curr < curr_size; index_curr++) // row
        {
            for (unsigned int index_prev = 0; index_prev < prev_size; index_prev++) // col
            {
                double dtx =
                    fabs(objects_msg_prev_.objects_vec[index_prev].x - objects_msg_curr.objects_vec[index_curr].x);
                double dty =
                    fabs(objects_msg_prev_.objects_vec[index_prev].y - objects_msg_curr.objects_vec[index_curr].y);
                double dtz =
                    fabs(objects_msg_prev_.objects_vec[index_prev].z - objects_msg_curr.objects_vec[index_curr].z);

                double dis_2 = dtx * dtx + dty * dty + dtz * dtz;

                costMatrix[index_curr][index_prev] = dis_2; // std::sqrt(dis_2);

                // todo iou
            }
        }

        HungarianAlgorithm HungAlgo;
        std::vector<int> assignment;
        double cost = HungAlgo.Solve(costMatrix, assignment);
      
        for (unsigned int index = 0; index < costMatrix.size(); index++)
        {

            if (assignment[index] >= 0 and costMatrix[index][assignment[index]] < 9) // todo:cost filter
            {
                objects_msg_curr.objects_vec[index].id = objects_msg_prev_.objects_vec[assignment[index]].id;
            }
            else
            {
                id_++;
                objects_msg_curr.objects_vec[index].id = id_;
            }
        }
  
        objects_msg_prev_ = objects_msg_curr;
    }
    else
    {
        id_ = 1;
        for (auto &object : objects_msg_curr.objects_vec)
        {
            object.id = id_;
            id_++;
        }
        objects_msg_prev_ = objects_msg_curr;
        first_flag_ = false;
    }

    /*1.align cooridinates to now*/

    /*2.data association*/

    /*3.kalman filter predict*/

    /*4.birth and death memory*/
}
} // namespace evolving_ad_ns
