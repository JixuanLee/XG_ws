#include <fstream>
// #include <signal.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>

std::ofstream ofs;          // 存储文件对象

int counter = 0;            // 路点计数
bool odom_flag = false;    // 判断是否接受到里程计信息
bool is_roslaunch = false;

float position_x = 0;       
float position_y = 0;
float position_x_old = 0;
float position_y_old = 0;   // 路点坐标

float node_res = 0.1; //0.2;    // 判断要存的两点之间距离是否太近
float dis_tmp = 0;


void odomCallback(const nav_msgs::Odometry & msg)
{
    position_x = msg.pose.pose.position.x;
    position_y = msg.pose.pose.position.y;
    
    odom_flag = true;
}

void timerCallback(const ros::TimerEvent &)
{
    if(odom_flag == true)           // 确定收到数据后再开始记录
    {
        dis_tmp = sqrt(pow(position_x_old - position_x, 2) + pow(position_y_old - position_y, 2));

        if (dis_tmp < node_res)     // 路径点间隔太小，不进行记录
            return;
        
        ofs << counter << " " << position_x << " " << position_y << std::endl;  //写入文件
        // ROS_INFO("Path point No.%d", counter);     //控制台打印
        
        counter++;
        position_x_old = position_x;
        position_y_old = position_y;
    }
    else
    {
        // ROS_INFO("waiting for odom msgs...");
    }
}

// 订阅车当前位置信息，然后进行每0.1秒判断是否到达新位置，如果到达新位置就打印当前信息，并将新位置的坐标点写入文件
// 该文件不做话题的发布，只负责写入本地txt中
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_record");
    ros::NodeHandle nh;
    ros::Subscriber OdomSub = nh.subscribe("/odom", 10, odomCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);   //计时器初始化,每0.1s进行检测当前点是否与原来的点发生位移，移动了就更新位置
    is_roslaunch = nh.param("path_record/is_roslaunch", false);
    if (is_roslaunch == true)
    {
        // roslaunch 启动，当前路径是在/home/user/.ros下
        ofs.open("../XG_ws/src/path_generator/paths/path_record.txt", std::ios::out);
    }
    else
    {
        // rosrun 启动，当前路径是在工作空间下
        ofs.open("src/path_generator/paths/path_record.txt", std::ios::out);
    }
    
    // itimerval tick;                         // 设置定时器结构体
    // memset(&tick, 0, sizeof(tick));         // 初始化为0
    // // Timeout to run first time
    // tick.it_value.tv_sec = 0;
    // tick.it_value.tv_usec = 500000;         // 0.5秒
    // // After first, the interval time for clock
    // tick.it_interval.tv_sec = 0;
    // tick.it_interval.tv_usec = 100000;      //其实就是从0.5s开始，每0.1s检测当前点是否与原来的点发生位移，移动了就更新位置

    // signal(SIGALRM, timerCallback);        // 定时触发函数
    // if(setitimer(ITIMER_REAL, &tick, NULL) < 0) // 失败返回-1
    // {
    //     ROS_INFO("Failed to set timer");
    //     return 0;
    // }
    
    ros::spin();
    ofs.close();
    return 0;
}