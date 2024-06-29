#include "ndt_localizer/map_loader.h"


MapLoader::MapLoader(ros::NodeHandle & nh)
{
    std::string pcd_file_name, map_topic;
    nh.param<std::string>("map_name", pcd_file_name, "");
    nh.param<std::string>("map_topic", map_topic, "pcd_points_map");
    map_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1, true); //仅发布一次


    // 获取地图的配置信息
    // double x_to_base_old = 0, y_to_base_old = 0;
    double longitude_old = 0, latitude_old = 0, altitude_old = 0;
    std::string yaml_path = ros::package::getPath("common").append("/map/config.yaml");
    YAML::Node yaml_config = YAML::LoadFile(yaml_path);
    if (pcd_file_name.empty())  // 自动检索pcd文件
    {
        if (yaml_config.size() > 0) // 使用最新的地图
        {
            int index = yaml_config.size() - 1;
            pcd_file_name = yaml_config[index]["name"].as<std::string>();
            // x_to_base_old = yaml_config[index]["x_to_base"].as<double>();
            // y_to_base_old = yaml_config[index]["y_to_base"].as<double>();
            longitude_old = yaml_config[index]["longitude"].as<double>();
            latitude_old  = yaml_config[index]["latitude"].as<double>();
            altitude_old  = yaml_config[index]["altitude"].as<double>();
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
                // x_to_base_old = yaml_config[i]["x_to_base"].as<double>();
                // y_to_base_old = yaml_config[i]["y_to_base"].as<double>();
                longitude_old = yaml_config[i]["longitude"].as<double>();
                latitude_old  = yaml_config[i]["latitude"].as<double>();
                altitude_old  = yaml_config[i]["altitude"].as<double>();
                has_found = true;
                break;
            }
        }
        if (!has_found)
        {
            ROS_ERROR_STREAM("load map configuration failed, please check pcd file name: " << pcd_file_name);
            int index = yaml_config.size() - 1;
            pcd_file_name = yaml_config[index]["name"].as<std::string>();
            // x_to_base_old = yaml_config[index]["x_to_base"].as<double>();
            // y_to_base_old = yaml_config[index]["y_to_base"].as<double>();
            longitude_old = yaml_config[index]["longitude"].as<double>();
            latitude_old  = yaml_config[index]["latitude"].as<double>();
            altitude_old  = yaml_config[index]["altitude"].as<double>();
            ROS_ERROR_STREAM("automatically load pcd file: " << pcd_file_name);
        }
    }
    

    // 发布地图部分
    std::string pcd_file_path = ros::package::getPath("common").append("/map/" + pcd_file_name);
    sensor_msgs::PointCloud2 pcd_msg_in;
    if (pcl::io::loadPCDFile(pcd_file_path.c_str(), pcd_msg_in) == -1) 
        ROS_ERROR_STREAM("load map failed: " << pcd_file_path);
    
    pcd_msg_in.header.frame_id = "map";
    if (pcd_msg_in.width != 0)
    {
		map_points_pub_.publish(pcd_msg_in);
        ROS_INFO("have load map and publish sucessfully!");
    }
    else
        ROS_ERROR("Empty PCD Map Output!");



    // 获取这次存行的信息，发布/map -> /odom的坐标变换
    // double x_to_base_now, y_to_base_now;
    double longitude_now, latitude_now, altitude_now;
    while (ros::ok())
    {
        if (nh.getParam("/gnss/longitude", longitude_now) && nh.getParam("/gnss/latitude", latitude_now) && nh.getParam("/gnss/altitude", altitude_now))
        {
            double X = 0, Y = 0;
            GPStoXY(longitude_now, latitude_now, longitude_old, latitude_old, X, Y);
            double Z = altitude_now - altitude_old;

            tfs.header.stamp = ros::Time::now();
            tfs.header.frame_id = "map";
            tfs.child_frame_id = "odom";
            tfs.transform.translation.x = X;
            tfs.transform.translation.y = Y;
            tfs.transform.translation.z = Z;
            tfs.transform.rotation.x = 0;
            tfs.transform.rotation.y = 0;
            tfs.transform.rotation.z = 0;
            tfs.transform.rotation.w = 1;
            broadcaster.sendTransform(tfs);
            ROS_INFO_ONCE("have load map configuration sucessfully!");
            break;
        }
        else
        {
            ROS_WARN_THROTTLE(1, "waiting for gnss location data ...");
            ros::Duration(0.1).sleep();
        }
    }
}

// 实现小范围内的经纬度变换成xy坐标形式
// 输入为目标经纬度和参考经纬度，均为degree
// 输出为xy，单位为m， xy方向满足东北天坐标系
void MapLoader::GPStoXY(double lon, double lat, double ref_lon, double ref_lat, double & X, double & Y)
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

// 存图时已经经过坐标变换了，加载时就不需要了
void MapLoader::TransformMap(sensor_msgs::PointCloud2 & in, sensor_msgs::PointCloud2 & out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in, *in_pcl);

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_(tf2_buffer_);
    geometry_msgs::TransformStamped transform = tf2_buffer_.lookupTransform("veh", "rslidar_link", ros::Time(0), ros::Duration(1.0));
    const Eigen::Affine3d veh_to_rslidar_affine = tf2::transformToEigen(transform);
    const Eigen::Matrix4f veh_to_rslidar_matrix = veh_to_rslidar_affine.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*in_pcl, *transformed_pcl, veh_to_rslidar_matrix);
    pcl::toROSMsg(*transformed_pcl, out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");
    ros::NodeHandle private_nh("~");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");
    MapLoader map_loader(private_nh);

    ros::spin();
    return 0;
}
