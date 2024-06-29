// 功能：跟随动态的uwb标签实时位置xy进行运动跟随
#include "uwb_follow/uwb_follow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_follow");
    ros::NodeHandle nh("~");
    UWB UWB(nh);
    ros::Rate loop_rate(UWB.RATE);

    while (ros::ok())
    {
        ros::spinOnce();
        
        // 用于判断是否接受到进行follow操作的使能。未接受时不进行操作，且不向总线send数据，避免can总线繁忙
        // if(UWB.FollowStart() == false)
        // {
        //     loop_rate.sleep();
        //     continue;
        // } // UWBtest

        // 用于检测各项数据接受是否正常的心跳
        if(UWB.CallbackFlag() == false)
        {
            loop_rate.sleep();
            continue;
        } // UWBtest


        UWB.GenLatCmd();
        UWB.GenLonCmd();
        UWB.PubCmd();
        UWB.PubBrake(); // UWBtest
        UWB.PubTarget(); // UWBtest

        loop_rate.sleep();
    }
    return 0;
}