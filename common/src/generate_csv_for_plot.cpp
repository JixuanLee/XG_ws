/*****************************************************************************************
功能：订阅目标话题，生成对应的csv文件，用以绘制图像，便于可视化了解数据
使用：1. 根据需求对本文件中的话题、数据类型、生成的csv格式等内容适当修改。
     2. 在保证目标话题有数据的情况下，启动 roslaunch common generate_csv_for_plot.launch
     3. 程序会在运行过程自动保存数据至csv，需要停止时在终端Ctrl+c停止即可。
     4. 参考 imugnss_cmd_plot.m 文件，绘制图像即可。
*****************************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "can_unpack/ImuGnss.h"

#include <fstream>
#include <queue>
#include <mutex>


std::ofstream imu_gnss_ofs;
std::queue<geometry_msgs::TwistStamped> imu_gnss_queue;
std::mutex imu_gnss_mutex;

std::ofstream cmd_ofs;
std::queue<geometry_msgs::TwistStamped> cmd_queue;
std::mutex cmd_mutex;


void ImuGnssCallback(const can_unpack::ImuGnss & msg)
{
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = msg.header.stamp;
    imu_gnss_mutex.lock();
    twist.twist.linear.x = std::sqrt((msg.twist.linear.x * msg.twist.linear.x) + (msg.twist.linear.y * msg.twist.linear.y));
    twist.twist.angular.z = msg.twist.angular.z;
    imu_gnss_queue.push(twist);
    imu_gnss_mutex.unlock();

    static int cnt = 0;
    ROS_INFO_THROTTLE(5, "imu_gnss counter: %d !", ++cnt);
}

void CmdCallback(const geometry_msgs::Twist & msg)
{    
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    cmd_mutex.lock();
    twist.twist = msg;
    cmd_queue.push(twist);
    cmd_mutex.unlock(); 

    static int cnt = 0;
    ROS_INFO_THROTTLE(5, "cmd counter: %d !", ++cnt);
}

void TimerCallback(const ros::TimerEvent &)
{
    geometry_msgs::TwistStamped imu_gnss_msg;
    imu_gnss_mutex.lock();
    if (!imu_gnss_queue.empty())
    {
        imu_gnss_msg = imu_gnss_queue.front();
        imu_gnss_queue.pop();
        imu_gnss_ofs << imu_gnss_msg.header.stamp    << ',' 
                     << imu_gnss_msg.twist.linear.x  << ','
                     << imu_gnss_msg.twist.angular.z << std::endl;
    }
    imu_gnss_mutex.unlock();


    geometry_msgs::TwistStamped cmd_msg;
    cmd_mutex.lock();
    if (!cmd_queue.empty())
    {
        cmd_msg = cmd_queue.front();
        cmd_queue.pop();
        cmd_ofs << cmd_msg.header.stamp    << ',' 
                << cmd_msg.twist.linear.x  << ','
                << cmd_msg.twist.angular.z << std::endl;
    }
    cmd_mutex.unlock();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "generate_csv");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    std::string file_path = ros::package::getPath("common").append("/csv/");
    std::string imu_gnss_file_path = file_path + "imu_gnss.csv";
    imu_gnss_ofs.open(imu_gnss_file_path, std::ios::trunc | std::ios::out);
    if (imu_gnss_ofs.is_open())
    {
        ROS_INFO("[%s] is opened!", imu_gnss_file_path.c_str());
        imu_gnss_ofs << "timestamp,v_x,w_z" << std::endl;
    }
    else
    {
        ROS_FATAL("[%s] failed to open!", imu_gnss_file_path.c_str());
        return -1;
    }
    std::string cmd_file_path = file_path + "cmd.csv";
    cmd_ofs.open(cmd_file_path, std::ios::trunc | std::ios::out);
    if (cmd_ofs.is_open())
    {
        ROS_INFO("[%s] is opened!", cmd_file_path.c_str());
        cmd_ofs << "timestamp,v_x,w_z" << std::endl;
    }
    else
    {
        ROS_FATAL("[%s] failed to open!", cmd_file_path.c_str());
        return -1;
    }

    ros::Subscriber imu_gnss_sub = nh.subscribe("imu_gnss", 10, ImuGnssCallback);
    ros::Subscriber cmd_sub = nh.subscribe("cmd_XUV25B", 10, CmdCallback);
    ros::Timer timer = nh.createTimer(loop_rate.expectedCycleTime(), TimerCallback);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    imu_gnss_ofs.close();
    cmd_ofs.close();
    return 0;
}
