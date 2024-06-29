#pragma once
#include <vector>
#include <queue>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

#include "common/utilities.h"

class UWB
{
public:
    float RATE;
    UWB(ros::NodeHandle & nh);

    // core
    bool FollowStart();
    bool CallbackFlag();
    void Stop();
    void Rear();
    void GenLatCmd();
    void GenLonCmd();
    void PubCmd();
    void PubTarget();
    void PubBrake();

private:    
    // 0
    // 0.0 心跳自己检测，判断tag是否断电
    std::chrono::steady_clock::time_point currentTime, lastUpdateTime;
    std::chrono::milliseconds elapsedTime;
    std::chrono::milliseconds filterDelay_Tag;      

    // 0.1 subscribe
    ros::Subscriber FollowEnableSub;
    void FollowEnableCallback(const std_msgs::Bool & msg);
    bool follow_enable_flag;
    ros::Subscriber UWBSub;
    void UWBCallback(const geometry_msgs::PointStamped & msg);
    bool UWB_flag;

    // 0.2 publish
    ros::Publisher CmdPub;
    std_msgs::Float32MultiArray cmd_msg;

    ros::Publisher BrakePub;
    std_msgs::Bool brake_msg;

    ros::Publisher TargetPub;
    geometry_msgs::PoseStamped uwb_target_msg;

    // 0.3 车辆相关
    // 0.3.1 vehicle硬件参数
    // float Veh_L;            //车辆轴距
    float Gap;              //前方两个基站(A0 A1)之间的距离
    float Target_L;         //目标最大轮廓直径尺寸
    float Dis_far;          //巡航距离，超出该距离后就按照该距离计算，即跟随判定距离不超过该值
    float Dis_stop;         //停车距离，进入该距离到刹车距离之间，减速停车
    float Dis_brake;        //刹车距离，进入该距离直接刹车，包含减速停车来不及的情况和倒车倒多的情况
    float Dis_rear;         //倒车距离，进入该距离后车辆将开始倒车，直至进入刹车距离，强制刹死
    float Angle_reverse;    //负角度判定，tag在车辆后方角度
    float Rear_angle_limit; //倒车时必须角度绝对值小于该值，防止离前车轮过近时倒车

    float Stop_linear_coefficient;  //停车时的速度衰退系数
    float Stop_linear_threshold;    //停车时的速度阈值，停车过程中速度小于此阈值时下发刹车信号
    float Stop_angular_coefficient; //停车时的角速度增益系数，使得在低速时产生较好的矫正方向效果

    // 0.3.2 车辆运动控制状态
    // 0.3.2.0
    VehicleState state_now, state_old;          //自身车辆状态
    float target_x_veh, target_y_veh;           //在车辆坐标系下目标点的坐标
    int Error_deque_size;                       //用于指定存放横向/纵向误差的容器大小
    
    // 0.3.2.1横向控制参数
    float Lat_P, Lat_I, Lat_D;  
    std::deque<float> lat_error_deque;
    float Angular_max, Angular_min, Angular_delta_max;

    // 0.3.2.2纵向控制参数
    float Lon_P, Lon_I, Lon_D;
    std::deque<float> lon_error_deque;
    float Linear_max, Linear_min, Linear_delta_max;  

    // 1.uwb结果滤波
    // 1.0
    std::deque<float> UWB_x, UWB_y; //存放滤波数据的容器
    int UWB_x_size, UWB_y_size;     //均值滤波xy队列容器大小
    float UWB_x_sum, UWB_y_sum;     //用于存放容器中数据点总和
    float UWB_x_veh, UWB_y_veh;     //得到容器中数据平均后的结果，也就是滤波后目标点在车辆坐标系下的xy值
    float Max_step_limit_x, Max_step_limit_y;
};
