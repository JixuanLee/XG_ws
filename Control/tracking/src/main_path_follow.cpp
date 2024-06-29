// TODO :   PID的起步逻辑（需要花窗滤波。）分开期望和PID2套。
// DEBUG：XG2024-05-11：发现使用map下的路径，存在

#include "tracking/path_follow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_follow");
    ros::NodeHandle nh("~");
    
    PathFollow path_follow(nh);

    ros::Rate loop_rate(path_follow.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!path_follow.CallbackFlag())
        {
            path_follow.PubCmd(true);
            loop_rate.sleep();
            continue;
        }

        path_follow.DynamicPredisAdjust();
        path_follow.FindTargetPoint();
        path_follow.GenLonCmd();
        path_follow.GenLatCmd();
        path_follow.UpdataVehicleStateData();
        path_follow.SteeringLinearAttenuation();
        // path_follow.StartingLinearAmplification(); // 仅在车辆底盘可提供有效反馈时使用。
        path_follow.HighSpeedAngularMaxAttenuation();
        path_follow.PubCmd();
        std::cout << std::endl;

        loop_rate.sleep();
    }
    return 0;
}