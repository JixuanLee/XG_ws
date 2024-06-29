#include <string>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

void readFile(nav_msgs::Path & path_msg, bool is_reverse_path, bool is_roslaunch)
{
    std::ifstream ifs;
    if (is_roslaunch == true)
    {
        // roslaunch 启动，当前路径是在/home/user/.ros下
        ifs.open("../XG_ws/src/path_generator/paths/path_record.txt", std::ios::in);        
    }
    else
    {
        // rosrun 启动，当前路径是在工作空间下
        ifs.open("src/path_generator/paths/path_record.txt", std::ios::in);        
    }

    if (ifs.is_open() == false)
    {
        ROS_ERROR("Path file doesn't exist...");
        exit(-1);
    }

    int path_num = 0;
    float position_x = 0;
    float position_y = 0;
    geometry_msgs::PoseStamped path_node;
    
    while (ifs >> path_num && ifs >> position_x && ifs >> position_y)
    {
        path_node.pose.position.x = position_x;
        path_node.pose.position.y = position_y;
        
        if (is_reverse_path == true)
        {
            path_msg.poses.insert(path_msg.poses.begin(), path_node);
        }
        else
        {
            path_msg.poses.push_back(path_node);
        }
    }
    ifs.close();

    if (path_msg.poses.size() > 0)
    {
        if (is_reverse_path == true)
        {
            ROS_INFO("Reading file successfully, generate reverse path.");
        }
        else
        {
            ROS_INFO("Reading file successfully, generate sequential path.");
        }
    }
    else
    {
        ROS_ERROR("Path file is empty!");
        exit(-1);
    }
}

float vehicle_x = 0;
float vehicle_y = 0;
float vehicle_yaw = 0;
float target_x_veh = 0;
float target_y_veh = 0;
float target_x = 0;
float target_y = 0;
geometry_msgs::PoseStamped path_node;
float dis_tmp = 0;
const float Dis_near = 0.05;   // 两个路径点过近时不会记录，这个值是阈值

void readUwb(nav_msgs::Path & path_msg)
{
    target_x = target_x_veh * cos(vehicle_yaw) - target_y_veh * sin(vehicle_yaw) + vehicle_x;
    target_y = target_x_veh * sin(vehicle_yaw) + target_y_veh * cos(vehicle_yaw) + vehicle_y;
    path_node.pose.position.x = target_x;
    path_node.pose.position.y = target_y;

    if (path_msg.poses.empty())
    {
        path_msg.poses.push_back(path_node);
    }
    else
    {
        uint vector_size = path_msg.poses.size(); 
        dis_tmp = sqrt(pow(path_msg.poses[vector_size - 1].pose.position.x - target_x, 2) + 
                       pow(path_msg.poses[vector_size - 1].pose.position.y - target_y, 2));
        if (dis_tmp > Dis_near)
        {
            path_msg.poses.push_back(path_node);
        }
    }
}

void odomCallback(const nav_msgs::Odometry & msg)
{
    vehicle_x = msg.pose.pose.position.x;
    vehicle_y = msg.pose.pose.position.y;
    vehicle_yaw = tf::getYaw(msg.pose.pose.orientation);
}

void uwbCallback(const geometry_msgs::PointStamped & msg)
{
    target_x_veh = msg.point.x;
    target_y_veh = msg.point.y;
}

// 读取文件中写好的path路径，然后发布。文件中path是从起点到终点，发布的msg容器中，poses首位是起点
// 或者通过uwb实时生成路径进行跟踪
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_generate");
    ros::NodeHandle nh;
    ros::Publisher PathPub = nh.advertise<nav_msgs::Path>("path_reference", 10);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    bool is_reverse_path = nh.param("path_generate/is_reverse_path", false);
    bool is_roslaunch = nh.param("path_generate/is_roslaunch", false);
    std::string mode = nh.param<std::string>("path_generate/mode", "file");
    float Rate;

    ros::Subscriber UwbSub = nh.subscribe("/uwb_process", 10, uwbCallback);             //记录目标在车辆坐标系下的xy
    ros::Subscriber OdomSub = nh.subscribe("/odom", 10, odomCallback);          //记录车辆在世界坐标系下的xy

    if (mode == "file")
    {
        readFile(path_msg, is_reverse_path, is_roslaunch);
        
        Rate = 1;
    }
    else if (mode == "uwb")
    {
        Rate = 20;
    }
    
    ros::Rate loop_rate(Rate);
    while (ros::ok())
    {  
        ros::spinOnce();

        if (mode == "uwb")
        {
            readUwb(path_msg);
        }
        path_msg.header.stamp = ros::Time::now();
        PathPub.publish(path_msg);

        loop_rate.sleep();
    }
    return 0;
}