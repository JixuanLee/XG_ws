#include "grid_cost_map_obs_cross/build_cost_map.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grid_cost_map_obs_cross");
    ros::NodeHandle nh("~");

    bool success = true;
    BuildCostMap Process(nh, success);
    if (!success)
    {
        ROS_ERROR("Init faild");
        return -1;
    }

    ros::spin();
    return 0;
}
