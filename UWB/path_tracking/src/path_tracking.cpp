#include "path_tracking/path_tracking.h"

//根据静态路径进行运动
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_tracking");
    ros::NodeHandle nh;
    PathTracking path_tracking(nh);
    ros::Rate loop_rate(path_tracking.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        if (path_tracking.CallbackFlag() == false)
        {
            loop_rate.sleep();
            continue;
        }

        path_tracking.PushPathNode();
        path_tracking.GenLatCmd();
        path_tracking.GenLonCmd();
        path_tracking.PubCmd();
        path_tracking.PubBrake();
        path_tracking.PubRear();

        path_tracking.ClearCallbackFlag();
        loop_rate.sleep();
    }
    return 0;
}