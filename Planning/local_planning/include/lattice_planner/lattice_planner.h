#ifndef __LATTICE_PLANNER__
#define __LATTICE_PLANNER__

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <eigen3/Eigen/Dense>

#include <vector>

// #define RATE 20
// #define OFFSET_NUM 5 // 一侧偏移个数
// #define OFFSET_DIS 2 // 每次偏移距离
// #define Pdt_len_max 120 // 最大预测点数
// #define Pdt_len_min 30 // 最少预测点数
// #define Pdt_len_step 10 // 预测点数步长
// #define PRE_DIS 3//5.0 // 基础预瞄距离
// #define ANGLE_MIN 0.00001 // 角速度死区
// #define U_MIN 1.0
// #define W_MAX 1.0
// #define Poly_Rank 3 // 拟合次数

class PathNode
{
public:
    float x, y, s, phi, c;
};

class VehState
{
public:
    float x, y, yaw, u, w;
};

class Lattice
{
public:
    Lattice(ros::NodeHandle &nh);

    float RATE;
    bool LatticeFlag(void);

    // publish
    void PubPdtPath(void);
    void PubPdtPath_Veh(void);
    void PubPathLocal_Veh(void);
    void PubOffsetRefPath_Veh(void);
    void PubLocalPathCluster(void);

    // 截取局部路径
    void GetLocalPath(void);
    int FindNearPoint(std::vector<PathNode> &path, VehState &veh);
    void PathWorld2Veh(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v);
    void PathVeh2World(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v);

    // 在车辆坐标系下进行碰撞检测
    bool ObsCheck(std::vector<PathNode> &path, int near_index);
    bool WhetherReplan(void);
    void LocalPathOffset(void);

    void Trim(std::vector<PathNode> &path); // 路径修剪
    void Polyfit(std::vector<PathNode> &path); // 拟合

    // 生成预测轨迹
    void GenTrajectory(void);
    void Track(std::vector<PathNode> &path, int len);
    void GenPathMsg(void);

private:
    // subscribe
    ros::Subscriber PathSub;
    void PathCallback(const nav_msgs::Path &msg);
    bool path_flag;
    bool replan_by_new_path;

    ros::Subscriber OdomSub;
    void OdomCallback(const nav_msgs::Odometry &msg);
    bool odom_flag;
    
    ros::Subscriber LocalMapSub;
    void LocalMapCallback(const nav_msgs::OccupancyGrid &msg);
    bool map_flag;
    
    // publish
    ros::Publisher PathPdtPub;
    nav_msgs::Path path_pdt_msg;

    ros::Publisher PdtPathVehPub;
    nav_msgs::Path path_pdt_veh_msg;

    ros::Publisher PathLocalVehPub;
    nav_msgs::Path path_local_veh_msg;

    ros::Publisher OffsetRefPathVehPub;
    nav_msgs::Path offset_ref_path_veh_msg;

    ros::Publisher LocalPathClusterPub;
    geometry_msgs::PoseArray LocalPathCluster_msg;

    // tf
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    // 地图
    cv::Mat image_map; // 二值化地图
    cv::Mat image_dis; // 距离地图
    int map_rows, map_cols; // 地图行数、列数
    float map_o_x, map_o_y; // 原点
    float map_res; // 分辨率

    // 路径
    std::vector<PathNode> path_ref;
    int near_index_ref_path;
    std::vector<PathNode> path_local, path_local_veh;
    std::vector<PathNode> path_pdt, path_pdt_veh;
    std::vector<std::vector<PathNode>> LocalPathCluster;
    int Path_Size_min;
    float Path_Length_min;

    // 车辆状态
    VehState state_now;

    // 碰撞检测
    float LocalMapHalfWidth;// 膨胀
    float OBS_DIS; // 碰撞检测距离
    float obs_path_len_min;
    float obs_path_len_max;

    // 路径偏移
    int OFFSET_NUM; // 一侧偏移个数
    float OFFSET_DIS; // 每次偏移距离

    // 路径拟合
    int Poly_Rank;
    Eigen::MatrixXd S; // 指数项矩阵
    Eigen::VectorXd A, B, P; // 系数列向量, 路点横/纵坐标列向量

    // 预瞄
    float Veh_L;   // 车辆轴距
    float PRE_DIS; // 基础预瞄距离
    float u_weight;
    int Pdt_len_max;  // 最大预测点数
    int Pdt_len_min;  // 最少预测点数
    int Pdt_len_step; // 预测点数步长
    float ANGLE_MIN;  // 角速度死区
    float U_MIN;
    float W_MAX;

    // 下标
    int path_local_index; // 局部路径下标
};

#endif // __LATTICE_PLANNER__