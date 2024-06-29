// TODO :   PID的起步逻辑（需要花窗滤波。）分开期望和PID2套。
// DEBUG：XG2024-05-11：发现使用map下的路径，存在

#include "tracking_obs_cross/path_follow_oc.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_follow_oc");
    ros::NodeHandle nh("~");
    
    PathFollowOC path_follow_oc(nh);

    ros::Rate loop_rate(path_follow_oc.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!path_follow_oc.CallbackFlag() || path_follow_oc.doPivotSteer || path_follow_oc.doObsCrossProcess) // 非正常的自主行驶时
        {

            if (path_follow_oc.doPivotSteer)  // 原地转向
            {
                path_follow_oc.PivotSteering();
            }
            else
            {
                if (path_follow_oc.doObsCrossProcess) // 开始越障
                {
                    path_follow_oc.ObsCrossProcess();
                }
                else // 其他情况，刹车
                    path_follow_oc.PubCmd(true);

            }

            loop_rate.sleep();
            continue;
        }

        path_follow_oc.DynamicPredisAdjust();
        path_follow_oc.FindTargetPoint();
        path_follow_oc.GenLonCmd();
        path_follow_oc.GenLatCmd();
        path_follow_oc.UpdataVehicleStateData();
        path_follow_oc.SteeringLinearAttenuation();
        path_follow_oc.PubCmd();
        std::cout << std::endl;

        loop_rate.sleep();
    }
    return 0;
}