#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sstream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>


bool has_saved_flag = false;
std::string filename;


std::string getCurrTime()
{
    // 将秒转换为日期和时间
    auto current_time = (ros::Time::now() + ros::Duration(8 * 60 * 60)).toBoost();
    int year   = current_time.date().year();
    int month  = current_time.date().month();
    int day    = current_time.date().day();
    int hour   = current_time.time_of_day().hours();
    int minute = current_time.time_of_day().minutes();
    int second = current_time.time_of_day().seconds();

    std::stringstream ss;
    ss << year << "-" << month << "-" << day << "_" << hour << "-" << minute << "-" << second;

    return ss.str();
}

// 存图的时候要变换坐标系
// 1. 首先针对legoloam的z轴朝前，变换回z轴朝上
// 2. 然后将点云坐标原点平移对齐到车辆坐标原点，而不是雷达坐标系的原点
// 3. 最后把点云坐标对齐到东北天坐标系下
void save_map(const sensor_msgs::PointCloud2 & msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *point_cloud_pcl);

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_(tf2_buffer_);
    geometry_msgs::TransformStamped transform = tf2_buffer_.lookupTransform("map", "vlidar_init", ros::Time(0), ros::Duration(1.0));
    const Eigen::Affine3d veh_to_rslidar_affine = tf2::transformToEigen(transform);
    const Eigen::Matrix4f veh_to_rslidar_matrix = veh_to_rslidar_affine.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pcl(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*point_cloud_pcl, *transformed_pcl, veh_to_rslidar_matrix);


    std::string pcd_filename = ros::package::getPath("common").append("/map/") + filename + ".pcd";
    pcl::io::savePCDFileASCII(pcd_filename, *transformed_pcl); 

    has_saved_flag = true;
    ROS_INFO_STREAM("point cloud map has saved to " << pcd_filename);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_saver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    filename = getCurrTime();

    // 保存地图配置信息
    // double x_to_base, y_to_base, longitude, latitude, altitude;
    // if (nh.getParam("/gnss/x_to_base", x_to_base) && nh.getParam("/gnss/y_to_base", y_to_base) && 
    //     nh.getParam("/gnss/longitude", longitude) && nh.getParam("/gnss/latitude", latitude) && nh.getParam("/gnss/altitude", altitude))
    double longitude, latitude, altitude;
    if (nh.getParam("/gnss/longitude", longitude) && nh.getParam("/gnss/latitude", latitude) && nh.getParam("/gnss/altitude", altitude))
    {
        std::string save_path_yaml = ros::package::getPath("common").append("/map/config.yaml");

        YAML::Emitter out;
        out << YAML::BeginSeq;
        out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << filename + ".pcd";
        // out << YAML::Key << "x_to_base" << YAML::Value << x_to_base;
        // out << YAML::Key << "y_to_base" << YAML::Value << y_to_base;
        out << YAML::Key << "longitude" << YAML::Value << longitude;
        out << YAML::Key << "latitude" << YAML::Value << latitude;
        out << YAML::Key << "altitude" << YAML::Value << altitude;
        out << YAML::EndMap;
        out << YAML::EndSeq;

        std::ofstream fout(save_path_yaml, std::ios::app);
        if (fout.is_open())
        {
            fout << out.c_str() << std::endl;
            fout.close();
            ROS_INFO_STREAM("map configuration has saved to " << save_path_yaml);
        } 
        else 
            ROS_WARN("fail to save map configuration!");
    }
    else
    {
        ROS_ERROR("cannot save map configuration due to no data!");
    }

    // 保存地图信息
    std::string point_cloud_topic_;
    if (private_nh.getParam("point_cloud_topic", point_cloud_topic_))
    {
        ros::Subscriber mapSub = nh.subscribe(point_cloud_topic_, 10, save_map);

        ros::Rate rate(20);
        while (has_saved_flag == false)
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        ROS_ERROR("cannot save map due to wrong topic name!");
    }
    
    return 0;
}
