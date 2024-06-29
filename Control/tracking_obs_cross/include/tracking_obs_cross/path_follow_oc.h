#ifndef __PATH_FOLLOW__
#define __PATH_FOLLOW__

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <deque>
#include <fstream>

// 描述车辆当前点的运动信息状态
struct VehicleState
{
    float x;                //在地图坐标系下的xy坐标，单位m
    float y;                //在地图坐标系下的xy坐标，单位m
    float z;                // 记录车辆当前在PCD点云地图中对应的z值（NDT估算）
    float yaw;              //车辆横摆角，单位rad
    float linear;           //车辆的速速，单位m/s
    float desired_linear_now; //期望的车辆巡航线速度
    float angular;          //车辆点横摆角速度，单位rad/s
    float desired_angular_now; //期望的车辆巡航角速度
    float theta;            //目标target相对于当前车辆的偏角
    float dis;              //目标target相对于当前车辆的距离

    //横向偏差
    float lat_error;        //单个偏差
    float lat_error_sum;    //累计偏差
    float lat_error_diff;   //偏差微分
    
    //纵向偏差
    float lon_error;
    float lon_error_sum;
    float lon_error_diff;
};

struct PathNode // 路点
{
    float x, y, s; // 坐标，起点到此路点的距离
};

struct DynamicParamsForPF // 通过launch接收的不同动态参数
{
    std::string name;

    float Lat_P, Lat_I, Lat_D;  
    float Angular_max_abs_for_LS; // 低速工况's 最大转向速度的绝对值，必须是正值，且较大。单位：rad/s，低速工况采用该“最大角速度”，中速工况采用衰减的“最大角速度”
    float Angular_max_abs_for_HS; // 高速工况's 最大转向速度的绝对值，必须是正值，且较小。单位：rad/s，高速工况采用该“最大角速度”
    float Low_linear_for_HS_w_attenuate; // 低速工况判据（下发的线速度Linear小于该值认为“低速工况”），较小值，必须为正（本质是绝对值）
    float High_linear_for_HS_w_attenuate; // 高速工况判据（下发的线速度Linear大于该值认为“高速工况”），Linear处于二者之间的，是“中速工况”，较大值，必须为正（本质是绝对值）
    float Angular_delta_max; // 角速度变化阈值，防止跳变

    float Lon_P, Lon_I, Lon_D;
    float Linear_max, Linear_min, Linear_delta_max;  

    float DESIRED_LINEAR;
    float DESIRED_MIN_LINEAR;
    float d_near;
    float d_stop;
    float k_ramp;
    float w_threshold_sharp_turn;
    float w_threshold_gentle_turn;

    float Target_L;         //目标最大轮廓直径尺寸
    float Veh_L; //车长
    float PRE_DIS;  // 基础预瞄距离
    float u_weight; //预瞄距离的速度影响系数，pre_dis = PRE_DIS + u_weight*V
};

class PathFollowOC
{
public:
    PathFollowOC(ros::NodeHandle &nh);
    void GetParams(ros::NodeHandle& nh);
    void GetDynamicParams(ros::NodeHandle& nh, DynamicParamsForPF &s);
    void UpdataDynamicParams(DynamicParamsForPF &sFrom);

    bool CallbackFlag(void);
    void DynamicPredisAdjust(void);
    void FindTargetPoint(void);
    void GenLonCmd(void);
    void GenLatCmd(void);
    float LonPID(double error);
    float LatPID(double error);
    void UpdataVehicleStateData(void);
    void SteeringLinearAttenuation(void); 
    void PivotSteering(void);
    void ObsCrossProcess(void);
    void Stop(void);
    void PubCmd(bool mustDoStop = false);

    bool get_transform(const std::string & target_frame_id, const std::string & source_frame_id,
                                 const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);
    std::string remove_first_slash(std::string frame_id);
    
    float RATE;
    geometry_msgs::Twist cmd;
    std_msgs::Bool brake;


    bool doPivotSteer;
    float aimPivotSteerAngle;

    bool doObsCrossProcess;
    float aimDisToMove;
    std_msgs::UInt8MultiArray ObsCtrlMode;

private:
    // subscribe
    ros::Subscriber OdomSub;
    std::string OdomSubTopic;
    void OdomCallback(const nav_msgs::Odometry &msg);

    ros::Subscriber PathSub;
    std::string PathSubTopic;
    void PathCallback(const nav_msgs::Path &msg);

    ros::Subscriber DesPointSub;
    std::string DesPointSubTopic;
    void DesPointCallback(const geometry_msgs::PoseStampedConstPtr &msgPtr);

    ros::Subscriber SceneModeSub;// 临时：为了XG验收，接收高速场景变换信号，后续可以整合为决策层接口
    std::string SceneModeSubTopic;
    void SceneModeCallback(const sensor_msgs::BatteryStateConstPtr &msgPtr);

    // publish
    ros::Publisher CmdPub;
    std::string  CmdPubTopic;

    ros::Publisher BrakePub;
    std::string  BrakePubTopic;

    ros::Publisher ObsCtrlModePub;
    std::string ObsCtrlModePubTopic;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    // flag
    bool odom_flag;
    bool path_flag;
    bool stop_flag;
    bool desPose_flag;
    bool isMapPath_flag;



    // path
    std::vector<PathNode> path_ref;
    geometry_msgs::PoseStamped des_pose_msg;

    // vehicle
    VehicleState state_now;
    VehicleState state_old;

    // dynamicParams
    DynamicParamsForPF HighSpeedParams;
    DynamicParamsForPF LowSpeedParams;
    DynamicParamsForPF UsingParams;


    // PID控制参数
    int Error_deque_size;  //用于指定存放横向/纵向误差的容器大小
    std::deque<float> lat_error_deque;
    std::deque<float> lon_error_deque;

    // 横向，预瞄
    int aim_num;    // 预瞄点下标
    float pre_dis; // 实际预瞄距离
    float aim_dis;  // 与预瞄点的距离
    float pre_angle; // 与预瞄点的夹角
    float pre_r;    // 预瞄半径

    // 仅为了XG高速自主行驶验收
    float k_p_linear_forXG;
    std::uint8_t scene_mode_forXG;
};

#endif // __PATH_FOLLOW__