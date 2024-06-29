/*****************************************************************************************
功能：消除GNSS天线安装误差，完成GNSS标定，修正由于稳态误差引起的点云地图转角偏差
使用：1. 启动 roslaunch common map_calibration.launch
        1.1 按需求填写要修改的pcd文件名（如果为空，则默认加载最新的pcd文件）
        1.2 是否通过bag标定，如果通过bag标定，则为true；如果利用车辆实时信息标定，则为false
     2. 启动节点后，会在rviz看到车辆在当前的pcd点云地图中的位置。根据车辆实际位置与现有位置的偏差，
        向服务端传入地图要旋转的角度。
        2.1 服务的话题为“/map_calibration”
        2.2 传入的参数有两个：
            2.2.1 delta_angle_deg：效果为将车辆沿着map的z轴正方向旋转的度数，单位deg；
                  也等价为点云地图中所有点在map坐标系下沿着z轴反方向旋转的度数，单位deg。
                  更新为增量式更新，每次只需要输入基于当前状态的更新参数即可。
            2.2.2 save_map：是否将当前旋转状态的点云更新至本地。如果为false，按照delta_angle_deg
                  参数进行点云地图更新；如果为true，忽略delta_angle_deg参数，将当前点云地图
                  保存至原路径，即覆盖。
        2.3 在终端使用 rosservice call /map_calibration "delta_angle_deg: 0.0 save_map: false" 
            更改相应的参数即可。
        2.4 服务端会返回两个参数，分别是deg/rad单位制下，目前点云地图相较于初始状态旋转的角度。

*****************************************************************************************/
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include "can_unpack/ImuGnss.h"
#include "common/MapCalibration.h"

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>

#include <mutex>
   
double longitude_ori = 0, latitude_ori = 0, altitude_ori = 0;
double trans_angle = 0;
std::mutex trans_angle_mutex;
pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_pcl(new pcl::PointCloud<pcl::PointXYZI>);
std::string pcd_file_path;

void GPStoXY(double lon, double lat, double ref_lon, double ref_lat, double & X, double & Y)
{
    const double CONSTANTS_RADIUS_OF_EARTH = 6371000.0;

    lon = lon / 180 * M_PI;
    lat = lat / 180 * M_PI;
    ref_lon = ref_lon / 180 * M_PI;
    ref_lat = ref_lat / 180 * M_PI;

    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double ref_sin_lat = std::sin(ref_lat);
    double ref_cos_lat = std::cos(ref_lat);
    double cos_d_lon = std::cos(lon - ref_lon);
    double sin_d_lon = std::sin(lon - ref_lon);

    double arg = std::max(1.0, std::min(-1.0, ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon));
    double c = std::acos(arg);

    double k = 1.0;
    if (std::abs(c) > 1e-3)
        k = (c / std::sin(c));

    X = float(k * cos_lat * sin_d_lon * CONSTANTS_RADIUS_OF_EARTH);
    Y = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
}

void save_pcd()
{
    Eigen::Affine3d transformation;
    trans_angle_mutex.lock();
    transformation = Eigen::AngleAxisd(-trans_angle, Eigen::Vector3d(0, 0, 1));
    trans_angle_mutex.unlock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pcd_pcl(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*pcd_pcl, *transformed_pcd_pcl, transformation);

    pcl::io::savePCDFileASCII(pcd_file_path, *transformed_pcd_pcl); 
    ROS_INFO_STREAM("modified point cloud map has saved to " << pcd_file_path);
}

// 用以补齐坐标变换，方便可视化
void imu_gnss_callback(const can_unpack::ImuGnss & msg)
{
    static tf2_ros::TransformBroadcaster dynamic_broadcaster;

    double X, Y, Z, yaw;
    GPStoXY(msg.longitude, msg.latitude, longitude_ori, latitude_ori, X, Y);
    Z = msg.altitude - altitude_ori;
    yaw = (90 - msg.head) / 180 * M_PI;
    if (yaw < -M_PI)
        yaw += 2 * M_PI;


    tf2::Transform tf;
    tf.setOrigin({X, Y, Z});
    tf.setRotation({{0, 0, 1}, yaw});
    tf2::Transform rotation;
    rotation.setOrigin({0, 0, 0});
    trans_angle_mutex.lock();
    rotation.setRotation({{0, 0, 1}, trans_angle});
    trans_angle_mutex.unlock();

    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = msg.header.stamp;
    tfs.header.frame_id = "map";
    tfs.child_frame_id = "veh";
    tfs.transform = tf2::toMsg(rotation * tf);

    dynamic_broadcaster.sendTransform(tfs);
}

bool map_calibration_callback(common::MapCalibration::Request & req, common::MapCalibration::Response & resp)
{
    if (req.save_map)
    {
        save_pcd();
        trans_angle_mutex.lock();
        resp.angle_rad = trans_angle;
        resp.angle_deg = trans_angle * M_1_PI * 180;
        trans_angle_mutex.unlock();
    }
    else
    {
        double delta_angle_deg = req.delta_angle_deg;
        double delta_angle_rad = delta_angle_deg / 180 * M_PI;
        trans_angle_mutex.lock();
        trans_angle += delta_angle_rad;
        if (trans_angle < -M_PI)
        {
            trans_angle += 2 * M_PI;
        }
        else if (trans_angle > M_PI)
        {
            trans_angle -= 2 * M_PI;
        }

        // ROS_INFO("current angle changed: %.3f rad, %.2f deg, sum angle changed:  %.3f rad, %.2f deg", 
        //          delta_angle_rad, delta_angle_deg, trans_angle, trans_angle * M_1_PI * 180);

        resp.angle_rad = trans_angle;
        resp.angle_deg = trans_angle * M_1_PI * 180;
        trans_angle_mutex.unlock();
    }
    
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_calibration");
    ros::NodeHandle nh("~");

    std::string pcd_file_name = nh.param<std::string>("map_name", "");
    std::string map_topic = nh.param<std::string>("map_topic", "/global_laser_cloud_map");
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1, true); //仅发布一次
    ros::Subscriber imu_gnss_sub = nh.subscribe("/imu_gnss", 10, imu_gnss_callback);
    ros::ServiceServer calibration_server = nh.advertiseService("/map_calibration", map_calibration_callback);

    // 获取地图的配置信息
    std::string yaml_path = ros::package::getPath("common").append("/map/config.yaml");
    YAML::Node yaml_config = YAML::LoadFile(yaml_path);
    if (pcd_file_name.empty())  // 自动检索pcd文件
    {
        if (yaml_config.size() > 0) // 使用最新的地图
        {
            int index = yaml_config.size() - 1;
            pcd_file_name = yaml_config[index]["name"].as<std::string>();
            longitude_ori = yaml_config[index]["longitude"].as<double>();
            latitude_ori  = yaml_config[index]["latitude"].as<double>();
            altitude_ori  = yaml_config[index]["altitude"].as<double>();
        }
        else
        {
            ROS_FATAL("there's no map configuration! failed to get map configuration automatically!");
            exit(-1);
        }
    }
    else    // 手动输入pcd文件
    {
        bool has_found = false;
        for (size_t i = 0; i < yaml_config.size(); ++i)
        {
            if (yaml_config[i]["name"].as<std::string>() == pcd_file_name)
            {
                longitude_ori = yaml_config[i]["longitude"].as<double>();
                latitude_ori  = yaml_config[i]["latitude"].as<double>();
                altitude_ori  = yaml_config[i]["altitude"].as<double>();
                has_found = true;
                break;
            }
        }
        if (!has_found)
        {
            ROS_ERROR_STREAM("load map configuration failed, please check pcd file name: " << pcd_file_name);
            int index = yaml_config.size() - 1;
            pcd_file_name = yaml_config[index]["name"].as<std::string>();
            longitude_ori = yaml_config[index]["longitude"].as<double>();
            latitude_ori  = yaml_config[index]["latitude"].as<double>();
            altitude_ori  = yaml_config[index]["altitude"].as<double>();
            ROS_ERROR_STREAM("automatically load pcd file: " << pcd_file_name);
        }
    }
    

    // 发布地图部分
    pcd_file_path = ros::package::getPath("common").append("/map/" + pcd_file_name);
    sensor_msgs::PointCloud2 pcd_msg_in;
    if (pcl::io::loadPCDFile(pcd_file_path.c_str(), pcd_msg_in) == -1) 
    {
        ROS_ERROR_STREAM("load map failed: " << pcd_file_path);
        exit(-1);
    }
    
    pcd_msg_in.header.frame_id = "map";
    if (pcd_msg_in.width != 0)
    {
		map_pub.publish(pcd_msg_in);
        pcl::fromROSMsg(pcd_msg_in, *pcd_pcl);
        ROS_INFO("have load map and publish sucessfully!");
    }
    else
    {
        ROS_ERROR("Empty PCD Map Output!");
        exit(-1);
    }

    ros::spin();
    return 0;
}