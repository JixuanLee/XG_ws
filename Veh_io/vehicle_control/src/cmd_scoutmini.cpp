// 此文件执行前请确保scout_mini小车的CAN硬件通讯已连接、ROS驱动已启动。
// 订阅：/cmd_raw，发布：/cmd_uwb_scoutmini（scoutmini的ROS驱动订阅）

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

// 自定义实际scout mini小车底盘运行最大线速度、最大角速度
#define MAX_LINEAR_SCOUTMINI  0.6       //0.15
#define MAX_ANGULAR_SCOUTMINI  0.35      //0.3

//自定义实际uwb程序规定的最大线速度、最大角速度（参考subxxxx.cpp中初始化进行修改）
#define MAX_LINEAR_UWB  (0.5 * (10/3.6))
#define MAX_ANGULAR_UWB (0.5 * (150*M_PI/180))

geometry_msgs::Twist cmd_uwb_msg_scoutmini;

ros::Publisher CmdPub_scoutmini;
ros::Subscriber CmdSub;

void RawCmdCallback(const std_msgs::Float32MultiArray::ConstPtr & msg)
{
    float u_tmp = msg->data[0]; //得到linear参考subxxxxx.cpp中初始化）
    float w_tmp = msg->data[1]; //得到angular（参考同上）

    cmd_uwb_msg_scoutmini.linear.x = u_tmp/MAX_LINEAR_UWB*MAX_LINEAR_SCOUTMINI;
    cmd_uwb_msg_scoutmini.angular.z = w_tmp/MAX_ANGULAR_UWB*MAX_ANGULAR_SCOUTMINI;

    CmdPub_scoutmini.publish(cmd_uwb_msg_scoutmini);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_scoutmini");
    ros::NodeHandle nh;

    CmdSub = nh.subscribe<std_msgs::Float32MultiArray>("/cmd_raw", 10, RawCmdCallback);
    CmdPub_scoutmini = nh.advertise<geometry_msgs::Twist>("/cmd_scoutmini", 10);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}



/* ----------------以下为测试，测试时对下方解注释，正常接收UWB数据对上方解注释，项目交付时删除下方---------------- */


// #include <iostream>
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <geometry_msgs/Twist.h>

// // 自定义实际scout mini小车底盘运行最大线速度、最大角速度
// #define MAX_LINEAR_SCOUTMINI  0.2 
// #define MAX_ANGULAR_SCOUTMINI  0.3

// //自定义实际uwb程序规定的最大torque、最大steer（参考subxxxx.cpp中初始化进行修改）
// // #define MAX_TURQUE_UWB  60
// // #define MAX_STEER_UWB 0.5236

// geometry_msgs::Twist cmd_uwb_scoutmini;

// ros::Publisher cmd_scoutmini_pub;
// ros::Subscriber raw_cmd_sub;

// void RawCmdCallback(const geometry_msgs::Twist & msg)
// {
//     float u_tmp = msg.linear.x; //得到键盘写入的v, -2 ~ 2 
//     float w_tmp = msg.angular.z;//得到w, -2 ~ 2

//     cmd_uwb_scoutmini.linear.x = u_tmp/2*MAX_LINEAR_SCOUTMINI;
//     cmd_uwb_scoutmini.angular.z = w_tmp/2*MAX_ANGULAR_SCOUTMINI;

//     cmd_scoutmini_pub.publish(cmd_uwb_scoutmini);

//     std::cout<<"cmd_uwb_scoutmini.linear.x="<<cmd_uwb_scoutmini.linear.x<<"\ncmd_uwb_scoutmini.angular.z = "<<cmd_uwb_scoutmini.angular.z<<"\n"<<std::endl;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "cmd_scoutmini_uwb_follow");
//     ros::NodeHandle nh;

//     raw_cmd_sub = nh.subscribe("/turtle1/cmd_vel", 10, RawCmdCallback);
//     cmd_scoutmini_pub = nh.advertise<geometry_msgs::Twist>("/cmd_uwb_scoutmini", 10);

//     ros::Rate loop_rate(20);
//     while(ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

// }
