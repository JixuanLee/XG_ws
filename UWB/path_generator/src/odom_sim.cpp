#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// 用于模拟生成一段odom曲线，方便后续测试
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_sim");
    ros::NodeHandle nh;
    ros::Publisher OdomPub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "veh";

    const double RATE = 20;
    ros::Rate loop_rate(RATE);

    double t = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        
        odom_msg.pose.pose.position.x = t;
        odom_msg.pose.pose.position.y = 0.5 * sin(10*t);
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan(5 * cos(10*t)));
        odom_msg.header.stamp = ros::Time::now();

        OdomPub.publish(odom_msg);

        t += M_PI_4 / RATE;
        loop_rate.sleep();
    }
    
    return 0;
}
