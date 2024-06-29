#include "path_tracking/path_tracking.h"

PathTracking::PathTracking(ros::NodeHandle & nh)
{
    // loop rate
    RATE = 20;

    // 0.心跳检测
    now_counter = 0;
    max_counter = 1 * RATE;

    // 1.subscribe
    OdomSub = nh.subscribe("/odom", 10, &PathTracking::OdomCallback, this);
    odom_flag = false;

    ChassisSub = nh.subscribe("/chassis", 10, &PathTracking::ChassisCallback, this);
    chassis_flag = false;
    
    PathSub = nh.subscribe("/path_reference", 10, &PathTracking::PathCallback, this);//读取文件或uwb发布的path路径
    path_flag = false;

    // 2.publish
    CmdPub = nh.advertise<std_msgs::Float32MultiArray>("/cmd_raw", 10);
    cmd_msg.data.resize(4);
    cmd_msg.data[0] = 0;    // 电机力矩
    cmd_msg.data[1] = 0;    // 前轮转角
    cmd_msg.data[2] = 1;    // > 0.5 自动模式，< 0.5 手动模式
    cmd_msg.data[3] = 0;    // > 0.5 需要紧急制动，< 0.5 不需要紧急制动

    //上帧车辆基本状态初始化
    state_old.dis = 0;
    state_old.theta = 0;
    state_old.linear = 0;
    state_old.angular = 0;

    //本帧车辆基本状态初始化
    state_now.dis = 0;
    state_now.theta = 0;
    state_now.linear = 0;
    state_now.angular = 0;

    // 3.跟踪路径
    // 3.2 车辆运动控制状态
    // 3.2.0 vehicle硬件参数
    Veh_L = 0.5; //3.5;    

    // 3.2.1 预瞄目标
    U_weight = 0.5;     //0.7;
    pre_dis_min = 0.1;  //0.2
    Near_dis = 0.15;     //0.2
    counter = 0;

    // 横向/纵向控制
    Error_deque_size = 3*RATE;
    // 3.2.2 横向控制
    Lat_P = 1.5;
    Lat_I = 0.04;
    Lat_D = 0.05;
    state_now.lat_error = 0;
    state_now.lat_error_sum = 0;
    state_now.lat_error_diff = 0;
    Angular_max = 120*M_PI/180;//30*M_PI/180;
    Angular_min = -120*M_PI/180;//30*M_PI/180;
    Angular_delta_max = 30*M_PI/180;
   
    // 3.2.3 纵向
    Lon_P = 15;//3
    Lon_I = 0.08;
    Lon_D = 0.01;
    state_now.lon_error = 0;
    state_now.lon_error_sum = 0;
    state_now.lon_error_diff = 0;
    Linear_start = 5 / 3.6;
    Linear_max = 10 / 3.6; 
    Linear_min = -7 / 3.6; 
    Linear_delta_max = 2 / 3.6;
}

void PathTracking::OdomCallback(const nav_msgs::Odometry & msg)
{
    state_now.x = msg.pose.pose.position.x;
    state_now.y = msg.pose.pose.position.y;
    state_now.yaw = tf::getYaw(msg.pose.pose.orientation);
    
    odom_flag = true;
}

void PathTracking::ChassisCallback(const std_msgs::Bool & msg)
{
    chassis_flag = msg.data;
}

void PathTracking::PathCallback(const nav_msgs::Path & msg)
{
    // 路点数目小于3，说明点数太少，或者说起点与终点距离很近，直接返回
    if (msg.poses.size() < 3)
    {
        path_flag = false;
        return;
    }
    
    // 获得数值
    path_ref.resize(msg.poses.size());
    for (uint i = 0; i < path_ref.size(); i++)
    {
        path_ref[i].x = msg.poses[i].pose.position.x;
        path_ref[i].y = msg.poses[i].pose.position.y;
    }
    
    // 计算路点的距离，切角，曲率
    for (uint i = 0; i < path_ref.size(); i++)
    {
        if (i == 0)
        {
            path_ref[i].s = 0;
            path_ref[i].phi = atan2(path_ref[i+1].y - path_ref[i].y, path_ref[i+1].x - path_ref[i].x);
        }
        else
        {
            path_ref[i].s = path_ref[i-1].s + sqrt(pow((path_ref[i].y - path_ref[i-1].y), 2) + pow((path_ref[i].x - path_ref[i-1].x), 2));
            if (i == path_ref.size() -1)
            {
                path_ref[i].phi = atan2(path_ref[i].y - path_ref[i-1].y, path_ref[i].x - path_ref[i-1].x);
            }
            else
            {
                path_ref[i].phi = atan2(path_ref[i+1].y - path_ref[i-1].y, path_ref[i+1].x - path_ref[i-1].x);
            }
            // phi_delta = path_ref[i].phi - path_ref[i-1].phi;

            // 将切向角度变化值限定在0～2pi
            // if (phi_delta > M_PI)
            // {
            //     phi_delta = phi_delta - 2*M_PI;
            // }
            // if (phi_delta <= -M_PI)
            // {
            //     phi_delta = phi_delta + 2*M_PI;
            // }
            
            // path_ref[i].c = phi_delta/(path_ref[i].s - path_ref[i-1].s);//曲率
        }
    }
    // path_ref[0].c = path_ref[1].c;
    

    // 根据路径点曲率计算参考车速
    // for (uint i = 0; i < path_ref.size(); i++)
    // {
    //     if (fabs(path_ref[i].c) >= C_max)
    //     {
    //         path_ref[i].u_ref = U_min;
    //     }
    //     else
    //     {
    //         path_ref[i].u_ref = (U_min - U_max)/pow(C_max, 2) * pow(fabs(path_ref[i].c), 2) + U_max;
    //         //(U_min - U_max)/C_max * fabs(path_ref[i].c) + U_max;
    //     }
    // }
    
    path_flag = true;
}
