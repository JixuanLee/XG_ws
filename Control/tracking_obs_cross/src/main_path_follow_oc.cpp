// TODO :   PID的起步逻辑（需要花窗滤波。）分开期望和PID2套。
// DEBUG：XG2024-05-11：发现使用map下的路径，存在

#include "tracking_obs_cross/path_follow_oc.h"

// 单独倒车
// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "path_follow_oc");
//     ros::NodeHandle nh("~");
    
//     PathFollowOC path_follow_oc(nh);

//     ros::Rate loop_rate(path_follow_oc.RATE);

//     const double aimdis = 3.0;

//     while (ros::ok())
//     {
//         static float nowDisToMove = 0.0;

//         if (nowDisToMove < aimdis)
//         {
//             ROS_INFO_ONCE("[3]Now start move backforward ! ");
//             path_follow_oc.cmd.linear.x = -0.5;
//             path_follow_oc.cmd.angular.z = 0;
//             path_follow_oc.brake.data = false;
//             path_follow_oc.PubCmd();
//             nowDisToMove += std::abs(path_follow_oc.cmd.linear.x)/path_follow_oc.RATE;
//         }
//         else
//         {
//             ROS_INFO_ONCE("[4]Now finish move backforward ! ");
//             path_follow_oc.cmd.linear.x = 0;
//             path_follow_oc.cmd.angular.z = 0;
//             path_follow_oc.brake.data = false;

//             ROS_INFO_ONCE("[5]Now start obs cross ! ");
//             path_follow_oc.ObsCtrlMode.data[1] = 1;
//             path_follow_oc.PubCmd();
//         }

//         loop_rate.sleep();
//     }
//     return 0;
// }


//先拐弯再倒车
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_follow_oc");
    ros::NodeHandle nh("~");
    
    PathFollowOC path_follow_oc(nh);

    ros::Rate loop_rate(path_follow_oc.RATE);

    const double turndis1 = 2.0; // 转向前的直线位移（经验值而非理论值）
    const double turndis2 = 4.0; // 转向的位移（经验值而非理论值）
    const double aimdis = 8.0; // 转向后的直线位移（经验值而非理论值）

    while (ros::ok())
    {
        static float nowDisToMove = 0.0;

        if (nowDisToMove < turndis1)
        {
            ROS_INFO_ONCE("[1]Now start move backforward ! ");
            path_follow_oc.cmd.linear.x = -0.7;
            path_follow_oc.cmd.angular.z = 0;
            path_follow_oc.brake.data = false;
            path_follow_oc.PubCmd();
            nowDisToMove += std::abs(path_follow_oc.cmd.linear.x)/path_follow_oc.RATE;
        }

        if (nowDisToMove >= turndis1 && nowDisToMove <= turndis2)
        {
            ROS_INFO_ONCE("[2]Now start trun ! ");
            path_follow_oc.cmd.linear.x = -0.5;
            path_follow_oc.cmd.angular.z = 0.5;
            path_follow_oc.brake.data = false;
            path_follow_oc.PubCmd();
            nowDisToMove += std::abs(path_follow_oc.cmd.linear.x)/path_follow_oc.RATE;
        }

        if (nowDisToMove > turndis2 && nowDisToMove < aimdis)
        {
            ROS_INFO_ONCE("[3]Now start move backforward ! ");
            path_follow_oc.cmd.linear.x = -0.7;
            path_follow_oc.cmd.angular.z = 0;
            path_follow_oc.brake.data = false;
            path_follow_oc.PubCmd();
            nowDisToMove += std::abs(path_follow_oc.cmd.linear.x)/path_follow_oc.RATE;
        }
        
        if (nowDisToMove >= aimdis)
        {
            ROS_INFO_ONCE("[4]Now finish move backforward ! ");
            path_follow_oc.cmd.linear.x = 0;
            path_follow_oc.cmd.angular.z = 0;
            path_follow_oc.brake.data = false;

            ROS_INFO_ONCE("[5]Now start obs cross ! ");
            path_follow_oc.ObsCtrlMode.data[1] = 1;
            path_follow_oc.PubCmd();
        }

        loop_rate.sleep();
    }

    return 0;
}