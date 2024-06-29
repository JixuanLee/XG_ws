#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <deque>

#include "common/utilities.h"

class PathTracking
{
public:
    float RATE;
    PathTracking(ros::NodeHandle & nh);
    
    // core
    bool CallbackFlag();
    void ClearCallbackFlag();
    void Stop();
    void Rear();
    void PushPathNode();
    void GenLatCmd();
    void GenLonCmd();
    void PubCmd();
    void PubBrake();
    void PubRear();

private:
    // 0 心跳检测
    uint16_t now_counter;       // 用于检测当前是否正常执行
    uint16_t max_counter;       // 如果0.5s未检测到链接，就退出程序

    // 1.subscribe
    ros::Subscriber OdomSub;
    void OdomCallback(const nav_msgs::Odometry & msg);
    bool odom_flag;
    ros::Subscriber ChassisSub;
    void ChassisCallback(const std_msgs::Bool & msg);
    bool chassis_flag;
    ros::Subscriber PathSub;
    void PathCallback(const nav_msgs::Path & msg);
    bool path_flag;

    // 2.publish
    ros::Publisher CmdPub;
    ros::Publisher BrakePub;
    ros::Publisher RearPub;

    std_msgs::Float32MultiArray cmd_msg;
    std_msgs::Bool brake_msg;
    std_msgs::Bool rear_msg;

    // 3.跟踪路径
    // 3.1 路径
    PathNode node_tmp;
    std::vector<PathNode> path_ref;

    // 3.2 车辆运动控制状态
    // 3.2.0 vehicle硬件参数
    float Veh_L;
    VehicleState state_now, state_old;     // 车辆在世界坐标系下坐标，速度，横摆角
    float dis_min, dis_tmp;
    
    // 3.2.1 预瞄目标并确定目标点
    float U_weight;     // 车速系数，用于确定预瞄距离
    float pre_dis;      // 预瞄距离
    float pre_dis_min;  // 用于确定预瞄过程中的最小距离
    int near_num;       // 最近点下标
    int aim_num;        // 通过预瞄点找到的目标点下标
    float aim_dis;      // 与目标点的距离
    float aim_angle;    // 与目标点的夹角
    float aim_radius;   // 行进至目标点过程中点曲率半径（阿克曼转向）
    float Near_dis;     // 用于判断车辆已经到达路径终点周围的距离阈值

    // 横向/纵向控制
    int counter;
    int Error_deque_size;
    // 3.2.2 横向
    float Lat_P, Lat_I, Lat_D;  
    std::deque<float> lat_error_deque;
    float Angular_max, Angular_min, Angular_delta_max;    

    // 3.2.3 纵向
    float Lon_P, Lon_I, Lon_D;
    std::deque<float> lon_error_deque;
    float Linear_start;
    float Linear_max, Linear_min, Linear_delta_max;
};
