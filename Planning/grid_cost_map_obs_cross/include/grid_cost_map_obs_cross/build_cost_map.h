#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_filters/MeanInRadiusFilter.hpp>
#include <filters/filter_chain.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using PointType = pcl::PointXYZI;


class BuildCostMap
{
private:
    ros::NodeHandle nh_;
    std::string global_laser_cloud_topic_;
    double global_map_res_;

    ros::Subscriber subGlobalLaserCloud;
    void GlobalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg);

    ros::Publisher pubGlobalGridMap;
    ros::Publisher pubGlobalOccupancyGridMap;
    grid_map::GridMap global_map;
    filters::FilterChain<grid_map::GridMap> globalMapFilterChain; //滤波器
    bool global_map_finished;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    pcl::PointCloud<PointType>::Ptr global_map_laser_cloud;

public:
    BuildCostMap(ros::NodeHandle & nh, bool & success);
    ~BuildCostMap();
};


