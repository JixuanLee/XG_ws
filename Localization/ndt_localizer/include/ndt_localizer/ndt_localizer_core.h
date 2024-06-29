#ifndef _NDT_CORE_H_
#define _NDT_CORE_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/kdtree.h>

class NdtLocalizer
{
public:
    NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~NdtLocalizer();

private:
    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber initial_pose_sub_;
    ros::Subscriber map_points_sub_;
    ros::Subscriber sensor_points_sub_;    
    ros::Subscriber gnss_sub_;
    ros::Publisher ndt_pose_pub_;

    std::string rviz_pose_topic_;
    std::string map_points_topic_;
    std::string current_points_topic_;
    std::string gnss_topic_;
    std::string ndt_pose_topic_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    // gnss
    bool use_gnss_;
    unsigned int gnss_update_time_;
    unsigned int gnss_update_time_now_;
    std::deque<double> gnss_bias_[3];// x, y, yaw
    double gnss_bias_average_[3] = {0.0, 0.0, 0.0};

    // init guess for ndt
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;

    std::string veh_frame_;
    std::string map_frame_;

    // ndt_对象保存了NDT算法的状态，包括目标点云，源点云，变换矩阵，参数，梯度，海森矩阵等。
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    Eigen::Matrix4f base_to_sensor_matrix_;
    Eigen::Matrix4f pre_trans, delta_trans;
    bool init_pose = false;

    double MAX_MEASUREMENT_RANGE = 120.0;
    double  voxel_leaf_size = 2.0;

    std::mutex ndt_map_mtx_;
    double converged_param_transform_probability_;
    std::map<std::string, std::string> key_value_stdmap_;

    pcl::search::KdTree<pcl::PointXYZ> kdtree;

    // function
    void init_params();

    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);
    void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_pointcurrent(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_gnss(const nav_msgs::Odometry & msg);

    pcl::PointXYZ findClosestPointInMap( const pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr, const pcl::PointXYZ& query_point);
    void removePointsByRange(pcl::PointCloud<pcl::PointXYZ> in_pcl, pcl::PointCloud<pcl::PointXYZ> out_pcl, double min_range, double max_range);
    void downsampler(const sensor_msgs::PointCloud2::ConstPtr& in, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & out);


    bool get_transform(const std::string & target_frame_id, const std::string & source_frame_id,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
                       const ros::Time & time_stamp);        
    bool get_transform(const std::string & target_frame_id, const std::string & source_frame_id,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);

    std::string remove_first_slash(std::string frame_id);
};
#endif