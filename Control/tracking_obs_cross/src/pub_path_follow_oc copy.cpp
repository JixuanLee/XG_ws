#include "tracking_obs_cross/path_follow_oc.h"

void PathFollowOC::PubCmd(bool mustDoStop)
{
    if (stop_flag || mustDoStop)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        brake.data = true;
    }
    else
        brake.data = false;
    
    ROS_INFO_THROTTLE(0.25, "[4-1] cmd_v = %.3f ; cmd_w = %.3f ; brake = %d", cmd.linear.x, cmd.angular.z, brake.data);
    ROS_INFO_THROTTLE(0.25, "[4-2] doPivotSteer = %d ; doObsCrossCmd = %d", ObsCtrlMode.data[0], ObsCtrlMode.data[1]);


    CmdPub.publish(cmd);
    BrakePub.publish(brake);
    ObsCtrlModePub.publish(ObsCtrlMode);

}