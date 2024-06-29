#include "uwb_follow/uwb_follow.h"

UWB::UWB(ros::NodeHandle & nh)
{
    //1.初始化变量
    RATE = 20; // 主频率

    // vehicle机械尺寸
    // Veh_L = 0.5;
    Gap       = nh.param("Gap", 1.8);
    Target_L  = nh.param("Target_L", 1.0);
    Dis_far   = nh.param("Dis_far", 12.0);
    Dis_stop  = Target_L + nh.param("Dis_stop_bias", 3.5);
    Dis_brake = Target_L + nh.param("Dis_brake_bias", 2.5);
    Dis_rear  = Target_L + nh.param("Dis_rear_bias", 1.5);
    Angle_reverse    = nh.param("Angle_reverse", 88.0) / 180 * M_PI;
    Rear_angle_limit = nh.param("Rear_angle_limit", 60.0) / 180 * M_PI;

    Stop_linear_coefficient  = nh.param("Stop_linear_coefficient", 0.75); 
    Stop_linear_threshold    = nh.param("Stop_linear_threshold", 0.5); 
    Stop_angular_coefficient = nh.param("Stop_angular_coefficient", 1.06); 

    //心跳自检uwb
    filterDelay_Tag = std::chrono::milliseconds(100);   //tag 断点100ms就掐断
    lastUpdateTime = std::chrono::steady_clock::now();

    //上帧车辆基本状态初始化
    state_old.dis = Target_L;
    state_old.theta = 0;
    state_old.linear = 0;
    state_old.angular = 0;

    //本帧车辆基本状态初始化
    state_now.dis = 0;
    state_now.theta = 0;
    state_now.linear = 0;
    state_now.angular = 0;

    //横向/纵向控制
    Error_deque_size = 3*RATE; 
    //横向控制PID、车辆状态初始化
    Lat_P = nh.param("Lat_P", 1.40);
    Lat_I = nh.param("Lat_I", 0.04);
    Lat_D = nh.param("Lat_D", 0.20);
    state_now.lat_error = 0;
    state_now.lat_error_sum = 0;
    state_now.lat_error_diff = 0;
    Angular_max = nh.param("Angular_max", 2.20);
    Angular_min = nh.param("Angular_min", -2.20);
    Angular_delta_max = nh.param("Angular_delta_max", 15.0) * M_PI / 180;

    //纵向控制PID、车辆状态初始化
    Lon_P = nh.param("Lon_P", 0.40);
    Lon_I = nh.param("Lon_I", 0.06);
    Lon_D = nh.param("Lon_D", 0.04);
    state_now.lon_error = 0;
    state_now.lon_error_sum = 0;
    state_now.lon_error_diff = 0;
    Linear_max = nh.param("Linear_max", 3.50);
    Linear_min = nh.param("Linear_min", -1.00);
    Linear_delta_max = nh.param("Linear_delta_max", 0.25);

    // uwb结果滤波
    UWB_x_size = 12; //滑窗滤波窗口大小
    UWB_y_size = 12; //滑窗滤波窗口大小
    Max_step_limit_x = 3;
    Max_step_limit_y = 3;

    //2.订阅
    FollowEnableSub = nh.subscribe("/follow_enable", 10, &UWB::FollowEnableCallback, this);
    follow_enable_flag = false;

    UWBSub = nh.subscribe("/uwb", 10, &UWB::UWBCallback, this);
    UWB_flag = false;

    //3.发布
    CmdPub = nh.advertise<std_msgs::Float32MultiArray>("/cmd_raw", 10);
    cmd_msg.data.resize(2);
    cmd_msg.data[0] = 0;    // 线速度
    cmd_msg.data[1] = 0;    // 角速度

    BrakePub = nh.advertise<std_msgs::Bool>("/brake", 5);
    brake_msg.data = false;

    TargetPub = nh.advertise<geometry_msgs::PoseStamped>("/uwb_target", 10);
    uwb_target_msg.header.frame_id = "veh";
}

void UWB::FollowEnableCallback(const std_msgs::Bool & msg)
{
    follow_enable_flag = msg.data;
}

void UWB::UWBCallback(const geometry_msgs::PointStamped & msg) 
{
    float raw_distance = sqrt(pow(msg.point.x, 2) + pow(msg.point.y - Gap / 2, 2));
    float raw_theta = atan2(msg.point.y - Gap / 2, msg.point.x);

    // if (UWB_x.empty() || UWB_y.empty())
    // {
    //     UWB_x.push_back(msg.point.x);
    //     UWB_y.push_back(msg.point.y - Gap / 2);

    //     UWB_x_veh = msg.point.x;
    //     UWB_y_veh = msg.point.y;
    // }
    // else if (fabs(msg.point.x - UWB_x.back()) <= Max_step_limit_x &&
    //          fabs(msg.point.y - UWB_y.back()) <= Max_step_limit_y)
    // {
        UWB_x.push_back(msg.point.x);
        // UWB_y.push_back(msg.point.y); // 如果是用uwb_process的数据，不需要偏移，拿到的已经是偏移后的数据
        UWB_y.push_back(msg.point.y - Gap / 2); //如果是用uwb本身解算出的坐标点，需要进行偏移

        if (UWB_x.size() > UWB_x_size) //在本文件初始化为6
        {
            UWB_x.pop_front();
        }

        if (UWB_y.size() > UWB_y_size)
        {
            UWB_y.pop_front();
        }

        UWB_x_sum = 0;
        UWB_y_sum = 0;

        for (int i = 0; i < UWB_x.size(); i++)
        {
            UWB_x_sum += UWB_x[i];
        }

        for (int i = 0; i < UWB_y.size(); i++)
        {
            UWB_y_sum += UWB_y[i];
        }

        UWB_x_veh = UWB_x_sum / UWB_x.size();
        UWB_y_veh = UWB_y_sum / UWB_y.size();
    // }

    // 如果不跳变，那么数据就会更新，否则延续上一次满足条件的数据
    target_x_veh = UWB_x_veh;
    target_y_veh = UWB_y_veh;      

    state_now.dis = sqrt(pow(target_x_veh, 2) + pow(target_y_veh, 2));
    if (state_now.dis > Dis_far)
        state_now.dis = Dis_far;
    state_now.theta = atan2(target_y_veh, target_x_veh);

    ROS_WARN_THROTTLE(1, "RAW distance: %.3f \t RAW theta: %.2f", raw_distance, raw_theta);
    ROS_INFO_THROTTLE(1, "tag distance: %.3f \t tag theta: %.2f", state_now.dis, state_now.theta);
    lastUpdateTime = std::chrono::steady_clock::now();
    UWB_flag = true;
}
