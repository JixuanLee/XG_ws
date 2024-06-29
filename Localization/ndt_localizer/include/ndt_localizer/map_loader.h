#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <yaml-cpp/yaml.h>

class MapLoader
{
public:
    MapLoader(ros::NodeHandle & nh);

private:
    ros::Publisher map_points_pub_;
    tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;

    void GPStoXY(double lon, double lat, double ref_lon, double ref_lat, double & X, double & Y);
    void TransformMap(sensor_msgs::PointCloud2 & in, sensor_msgs::PointCloud2& out);
}; //MapLoader

#endif