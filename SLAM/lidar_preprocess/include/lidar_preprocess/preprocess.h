#ifndef PREPROCESSOR_HPP
#define PREPROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_preprocess/read_yaml.h"


class Preprocessor
{
public:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    int isLidarGotIn[4]={0};

    // Eigen::Matrix4d T_middle2veh = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_front2middle_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_left2middle_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_right2middle_ = Eigen::Matrix4d::Identity();

    //去过近点
    double min_dist_;

    //去地面
    double leaf_size_;
    double cell_size_;
    double z_threshold_;
    double y_threshold_;
    double x_threshold_;
    double ground_remove_threshold_;
    double ground_z_;
    double ground_beyond_threshold_;

    //全局的参数
    PointCloudT::Ptr middle_cloud_ptr_;
    PointCloudT::Ptr front_cloud_ptr_;
    PointCloudT::Ptr left_cloud_ptr_;
    PointCloudT::Ptr right_cloud_ptr_;
    PointCloudT::Ptr all_cloud_ptr_;

    ros::NodeHandle nh_;

    ros::Publisher ros_concat_points_pub_; // （若为多雷达）拼接后的点云（原始点云）, I , I -> A、B、C
    ros::Publisher ros_cliped_all_points_pub_; // （拼接后）预处理后的点云（含地面点）, A，A=B+C
    ros::Publisher ros_cliped_without_ground_points_pub_; // （拼接后）预处理后的点云（不含地面点）, B，A=B+C
    ros::Publisher ros_ground_points_pub_; // （拼接后）预处理后的地面点云, C，A=B+C

    typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::PointCloud2,sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
        syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mf_front_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mf_left_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mf_right_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mf_middle_sub;
    ros::Subscriber points_middle_sub_;

    std::string middle_lidar_sub_topic_;
    std::string front_lidar_sub_topic_;
    std::string left_lidar_sub_topic_;
    std::string right_lidar_sub_topic_;

    ros::Time LidarTimeNow[4] = {ros::Time::now()};

    void Run();
    void MultiLidarCallback(const sensor_msgs::PointCloud2ConstPtr &front, const sensor_msgs::PointCloud2ConstPtr &left,
                            const sensor_msgs::PointCloud2ConstPtr &right, const sensor_msgs::PointCloud2ConstPtr &middle);
    void SingleLidarCallback(const sensor_msgs::PointCloud2ConstPtr &middle);


    void TransformCloud2TargetLink(PointCloudT::Ptr in_cloud, Eigen::Matrix4d lidar_to_base_link);
    void TransPubPCL2ROS(PointCloudT::Ptr &input_cloud, ros::Publisher ros_pub);
    void ClipCloud(const PointCloudT::Ptr in_cloud_ptr, PointCloudT::Ptr out_all_cloud_ptr, PointCloudT::Ptr out_without_ground_cloud_ptr, PointCloudT::Ptr out_ground_cloud_ptr);

    Preprocessor();
    ~Preprocessor();

private:
    ros::NodeHandle node_handle_, private_node_handle_;
    ReadYaml *m_read_dynamic_yaml_;
    std::string staticyaml_dir_;
    bool isMultiLidar;

    pcl::RadiusOutlierRemoval<PointT > inilerFilterWithoutGround;
    pcl::RadiusOutlierRemoval<PointT > inilerFilterGround;

    float size_L;
    float size_W ; 
    float size_cell ;
    
    int ** cell_isCover ;
    float ** cell_max_Z;
    float ** cell_min_Z ;
    int ** cell_not_ground ;

    void ReadStaticYaml(string dir);
};

#endif  // PREPROCESSOR_HPP
