// 订阅：/cmd_raw，发布：/cmd_XUV25B

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_msg_XUV25B;

ros::Publisher CmdPub_XUV25B;
ros::Subscriber CmdSub;

void RawCmdCallback(const std_msgs::Float32MultiArray::ConstPtr & msg)
{
    float u_tmp = msg->data[0]; //得到linear参考subxxxxx.cpp中初始化）
    float w_tmp = msg->data[1]; //得到angular（参考同上）

    cmd_msg_XUV25B.linear.x = u_tmp;
    cmd_msg_XUV25B.angular.z = w_tmp;

    CmdPub_XUV25B.publish(cmd_msg_XUV25B);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_XUV25B");
    ros::NodeHandle nh;

    CmdSub = nh.subscribe<std_msgs::Float32MultiArray>("/cmd_raw", 10, RawCmdCallback);
    CmdPub_XUV25B = nh.advertise<geometry_msgs::Twist>("/cmd_XUV25B", 10);

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
