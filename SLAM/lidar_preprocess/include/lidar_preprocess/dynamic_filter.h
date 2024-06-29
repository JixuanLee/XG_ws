#ifndef __DYNAMIC_FILTER_HPP
#define __DYNAMIC_FILTER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// OMP
#include <omp.h>


class DynamicFilter
{
public:
    ros::NodeHandle nh_;

    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
    
    pcl::VoxelGrid<PointXYZI> downSizeFilterStatic;


    class GridCell
    {
    public:
        double log_odds; //占有率

        GridCell(void)
        {
            log_odds = 0;
        }

        double get_occupancy(void)
        {
            return 1.0 / (1 + exp(-log_odds));
        }

        double get_log_odds(void)
        {
            return log_odds;
        }

        void add_log_odds(double lo)
        {
            log_odds += lo;
        }

    private:

    };

    typedef std::vector<GridCell> OccupancyGridMap; // 占有率地图本质上就是一个线性vector容器，内存占有率数据

    DynamicFilter(void);
    ~DynamicFilter();

    void callback(const sensor_msgs::PointCloud2ConstPtr&, const nav_msgs::OdometryConstPtr&);
    void callback_do_murge_ground(const sensor_msgs::PointCloud2ConstPtr&, const sensor_msgs::PointCloud2ConstPtr&, const nav_msgs::OdometryConstPtr&);
    void input_cloud_to_occupancy_grid_map(const CloudXYZIPtr&);
    void devide_cloud(const CloudXYZIPtr&, CloudXYZIPtr&, CloudXYZIPtr&);
    void down_size_filter(CloudXYZIPtr&);

    int get_index_from_xy(const double x, const double y)
    {
        const int _x = floor(x / resolution_ + 0.5) + grid_width_2_; //四舍五入（规整）+半个总栅格地图边长（取正）
        const int _y = floor(y / resolution_ + 0.5) + grid_width_2_;
        return _y * grid_width_ + _x;
    }

    int get_x_index_from_index(const int index)
    {
        return index % grid_width_;
    }

    int get_y_index_from_index(const int index)
    {
        return index / grid_width_;
    }

    double get_x_from_index(const int);
    double get_y_from_index(const int);
    void publish_occupancy_grid_map(const ros::Time&, const std::string&);
    std::string remove_first_slash(std::string);

    // 在总的栅格地图上，能找到这个点xy所处的栅格，就是valid点
    bool is_valid_point(double x, double y)
    {
        const int index = get_index_from_xy(x, y);
        if(x < -width_2_ || x > width_2_ || y < -width_2_ || y > width_2_){
            return false;
        }else if(index < 0 || grid_num_ <= index){
            return false;
        }else{
            return true;
        }
    }

    void transform_occupancy_grid_map(const Eigen::Vector2d&, double, OccupancyGridMap&);
    void set_clear_grid_cells(const std::vector<double>&, const std::vector<bool>&, OccupancyGridMap&);
    void start_process(void);

private:
    double resolution_; // 分辨率
    double width_;      // 总的地图的边长尺寸
    double width_2_;    // 总的地图的边长尺寸/2
    int grid_width_;    //总地图的每个边上栅格的数量
    int grid_width_2_;  //总地图的每个边上栅格的数量/2
    int grid_num_;      //总地图的栅格的数量
    double occupancy_threshold_;//动态点判定的占有率阈值
    int beam_num_;      // 分割的光束数量
    double log_odds_increase_;  // 占有率增量delta
    double log_odds_decrease_;  // 占有率负增量delta
    std::string input_pointcloud_topic_;
    std::string input_pointcloud_ground_topic_;
    std::string input_odom_topic_;
    bool doMurgeTheGroundPoints_;


    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    ros::NodeHandle local_nh_;
    ros::Publisher dynamic_pub_;
    ros::Publisher static_pub_;
    ros::Publisher static_pub_pcl_;
    ros::Publisher grid_pub_;

    OccupancyGridMap occupancy_grid_map_;// 存储上一帧的ogm数据
};

#endif// __DYNAMIC_FILTER_HPP
