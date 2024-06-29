#include "uwb_follow/uwb_follow.h"

void UWB::PubCmd()
{
    CmdPub.publish(cmd_msg); // UWBtest
    if (cmd_msg.data.size() >= 2)
    {
        ROS_INFO_THROTTLE(1, "linear: %.2f  angular: %.2f\n", cmd_msg.data[0], cmd_msg.data[1]);
    }
}

void UWB::PubTarget()
{
    uwb_target_msg.pose.position.x = target_x_veh;
    uwb_target_msg.pose.position.y = target_y_veh;
    uwb_target_msg.pose.orientation = tf::createQuaternionMsgFromYaw(state_now.theta); //不是目标点自身的角度，而是目标点与车辆原点的夹角
    uwb_target_msg.header.stamp = ros::Time::now();

    TargetPub.publish(uwb_target_msg);
}

void UWB::PubBrake()
{
    BrakePub.publish(brake_msg); // brake_msg.data = false/ture
}