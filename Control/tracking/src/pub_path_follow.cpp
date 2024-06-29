#include "tracking/path_follow.h"

void PathFollow::PubCmd(bool mustDoStop)
{
    if (stop_flag || mustDoStop)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        brake.data = true;
    }
    else
        brake.data = false;
    
    ROS_INFO_THROTTLE(0.25, "[4] cmd_v = %.3f ; cmd_w = %.3f", cmd.linear.x, cmd.angular.z);
    
    CmdPub.publish(cmd);
    BrakePub.publish(brake);
}