#include "grid_cost_map_obs_cross/build_cost_map.h"


BuildCostMap::BuildCostMap(ros::NodeHandle & nh, bool & success)
: nh_(nh), globalMapFilterChain("grid_map::GridMap"), listener_(buffer_)
{   
    // 配置ros参数
    global_laser_cloud_topic_ = nh_.param("global_laser_cloud_topic", std::string("/global_laser_cloud_map"));
    global_map_res_ = nh_.param("global_map_res_", 0.2);

    subGlobalLaserCloud = nh_.subscribe(global_laser_cloud_topic_, 1, &BuildCostMap::GlobalLaserCloudCallback, this);
    pubGlobalGridMap = nh_.advertise<grid_map_msgs::GridMap>("global_grid_map", 1, true);
    pubGlobalOccupancyGridMap = nh_.advertise<nav_msgs::OccupancyGrid>("global_occupancy_grid_map", 1, true);

    global_map.setFrameId("/map");
    global_map.add("elevation");
    global_map.add("elevation_low");
    global_map.add("elevation_high");
    global_map_finished = false;

    // 配置滤波器
    if (!globalMapFilterChain.configure("global_map_filters", nh_))
    {
        ROS_ERROR("Could not configure the global filter chain!");
        success = false;
        return;
    }

    // 点云初始化
    global_map_laser_cloud.reset(new pcl::PointCloud<PointType>);

    success = true;
}

BuildCostMap::~BuildCostMap()
{
}

void BuildCostMap::GlobalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg)
{
    // 初始化grid_map
    global_map_laser_cloud->clear();
    pcl::fromROSMsg(*msg, *global_map_laser_cloud);

    PointType min_value, max_value;
    pcl::getMinMax3D(*global_map_laser_cloud, min_value, max_value);

    grid_map::Position oriPoint;
    oriPoint.x() = (min_value.x + max_value.x) / 2;
    oriPoint.y() = (min_value.y + max_value.y) / 2;

    global_map.setTimestamp(msg->header.stamp.toNSec());
    global_map.setGeometry(grid_map::Length(max_value.x - min_value.x + 2, max_value.y - min_value.y + 2), global_map_res_, oriPoint);
    ROS_INFO("Created global map with size %f x %f m (%i x %i cells).",
             global_map.getLength().x(), global_map.getLength().y(),
             global_map.getSize()(0), global_map.getSize()(1));


    // 高程图elevation
    // 通过某一个栅格内最高和最低的点云确定，没有点云的栅格值为nan
    ros::Time start = ros::Time::now();
    for (int i = 0; i < global_map_laser_cloud->points.size(); i++)
    {
        PointType p = global_map_laser_cloud->points[i];
        grid_map::Position pos;
        pos.x() = p.x;
        pos.y() = p.y;

        if (global_map.atPosition("elevation_low", pos) > p.z || isnan(global_map.atPosition("elevation_low", pos)))
        {
            global_map.atPosition("elevation_low", pos) = p.z;
        }
        if (global_map.atPosition("elevation_high", pos) < p.z || isnan(global_map.atPosition("elevation_high", pos)))
        {
            global_map.atPosition("elevation_high", pos) = p.z;
        }
    }
    for (grid_map::GridMapIterator it(global_map); !it.isPastEnd(); ++it)
    {
        if (!isnan(global_map.at("elevation_low", *it)))
        {
            global_map.at("elevation", *it) = global_map.at("elevation_high", *it) - global_map.at("elevation_low", *it);
        }
    }
    // global_map.erase("elevation_low");
    // global_map.erase("elevation_high");
    ros::Time end = ros::Time::now();
    ROS_INFO("elevation finished!, uesed %f s", (end - start).toSec());


    // 使用yaml中的滤波链进行滤波，得到可通行度traversability
    // 具体流程和内容如下：
    // 1. 对高程图使用中值滤波，填补空洞，得到“elevation_fill”图层
    // 2. 利用“elevation_fill”图层计算坡度，得到“slope”图层
    // 3. 用均值滤波使高程图光滑，得到“elevation_smooth”图层，然后计算滤波前后高程差值的绝对值，得到粗糙度“roughness”图层
    // 4. 用坡度的方差表示边界代价，得到“edges”图层
    // 5. 由 坡度、粗糙度、边界代价 加权得到的可通行度“traversability”图层
    start = ros::Time::now();
    if (!globalMapFilterChain.update(global_map, global_map))
    {
        ROS_ERROR("Could not update the global map filter chain!");
        return;
    }
    else
    {
        global_map.erase("normal_vectors_x");
        global_map.erase("normal_vectors_y");

        end = ros::Time::now();
        ROS_INFO("filter chain finished!, uesd %f s", (end - start).toSec());
    }
    

    // 后续处理
    for (grid_map::GridMapIterator it(global_map); !it.isPastEnd(); ++it)
    {
        // 1. 中值滤波补洞后的高程图中，没有激光雷达点地方，认为是为未探明区域
        if (isnan(global_map.at("elevation_fill", *it)))
        {
            if (!isnan(global_map.at("edges", *it)))
                global_map.at("traversability", *it) = 0.3 * global_map.at("edges", *it);
            else
                global_map.at("traversability", *it) = NAN;
        }
    }

    // grid_map转成对应ros message
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(global_map, grid_map_msg);
    pubGlobalGridMap.publish(grid_map_msg);

    nav_msgs::OccupancyGrid occupancy_grid_map_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(global_map, "traversability", 0, 1, occupancy_grid_map_msg);
    pubGlobalOccupancyGridMap.publish(occupancy_grid_map_msg);

    global_map_finished = true;
}
