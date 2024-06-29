#include "tracking_obs_cross/path_follow_oc.h"

PathFollowOC::PathFollowOC(ros::NodeHandle& nh): listener_(buffer_)
{
    GetParams(nh);

    // subscribe

    // publish
    CmdPub = nh.advertise<geometry_msgs::Twist>(CmdPubTopic, 10);//控制指令
    BrakePub = nh.advertise<std_msgs::Bool>(BrakePubTopic, 10); // 刹车信号
    
    ObsCtrlModePubTopic = "/obs_cross_ctrl_mode";
    ObsCtrlModePub = nh.advertise<std_msgs::UInt8MultiArray>(ObsCtrlModePubTopic, 10); // 越障相关话题，To: 底盘CAN总线

    // flag
    odom_flag = false;
    path_flag = false;
    // stop_flag = true;
    desPose_flag = false;

    //  TODO:排查下列参数有无优化空间。

    // loop rate
    RATE = 20;

    //上帧车辆基本状态初始化
    state_old.dis = 0; // Target_L
    state_old.theta = 0;
    state_old.linear = 0;
    state_old.angular = 0;

    //本帧车辆基本状态初始化
    state_now.dis = 0;
    state_now.theta = 0;
    state_now.linear = 0;
    state_now.angular = 0;

    //横向/纵向控制
    Error_deque_size = 3*RATE; 
    
    //车辆状态初始化
    state_now.lat_error = 0;
    state_now.lat_error_sum = 0;
    state_now.lat_error_diff = 0;
    state_now.lon_error = 0;
    state_now.lon_error_sum = 0;
    state_now.lon_error_diff = 0;

    // 越障相关
    doPivotSteer = false;
    doObsCrossProcess = false;

    ObsCtrlMode.layout.dim.resize(2); 
    ObsCtrlMode.layout.dim[0].label = "DoPivotSteer";
    ObsCtrlMode.layout.dim[0].size = 1; // 第一个维度的大小
    ObsCtrlMode.layout.dim[0].stride = 2; // 第一个维度的跨度
    ObsCtrlMode.layout.dim[1].label = "DoAutoObsCross";
    ObsCtrlMode.layout.dim[1].size = 2; // 第二个维度的大小
    ObsCtrlMode.layout.dim[1].stride = 1; // 第二个维度的跨度
    ObsCtrlMode.layout.data_offset = 0;

    ObsCtrlMode.data.resize(2); 
    ObsCtrlMode.data[0] = 0; // 原地转向使能, 0：关闭；1：开启
    ObsCtrlMode.data[1] = 0; // 自主越障使能, 0：关闭；1：开启

}

void PathFollowOC::GetParams(ros::NodeHandle& nh)
{
    nh.param("OdomSubTopic", OdomSubTopic, std::string("/odom_by_imu_gnss"));
    nh.param("PathSubTopic", PathSubTopic, std::string("/local_planning/pdt_path_in_veh")); // 默认使用veh局部坐标系下的路径信息。也可以使用map下的路径，但请先修改部分代码。
    nh.param("DesPointSubTopic", DesPointSubTopic, std::string("/move_base_simple/goal"));
    nh.param("CmdPubTopic", CmdPubTopic, std::string("/cmd_XUV25B"));
    nh.param("BrakePubTopic", BrakePubTopic, std::string("/brake"));
    nh.param("isMapPath_flag", isMapPath_flag, true);

    nh.param("aimPivotSteerAngle", aimPivotSteerAngle, float(285.0));
    nh.param("aimDisToMove", aimDisToMove, float(0.2));


    // HighSpeedParams.name = "HighSpeedParams"; // 此name需与yaml中的一级命名空间保持一致
    LowSpeedParams.name = "LowSpeedParams";
    UsingParams.name = "UsingParams";

    GetDynamicParams(nh, LowSpeedParams);

    UpdataDynamicParams(LowSpeedParams);

}

void PathFollowOC::GetDynamicParams(ros::NodeHandle& nh, DynamicParamsForPF &s)
{
    std::string sNameSpace = std::string("/path_follow_oc/") + s.name;

    s.Lat_P = 2.6;    if(!nh.getParam(sNameSpace+"/"+"Lat_P",s.Lat_P)){ std::cout<<"\033[31mcant get param Lat_P\033[0m"<<std::endl;exit(0); }
    s.Lat_I = 0.06;    if(!nh.getParam(sNameSpace+"/"+"Lat_I",s.Lat_I)){ std::cout<<"\033[31mcant get param Lat_I\033[0m"<<std::endl;exit(0); }
    s.Lat_D = 0.06;    if(!nh.getParam(sNameSpace+"/"+"Lat_D",s.Lat_D)){ std::cout<<"\033[31mcant get param Lat_D\033[0m"<<std::endl;exit(0); }
    s.Angular_max_abs_for_LS = 2.1;    if(!nh.getParam(sNameSpace+"/"+"Angular_max_abs_for_LS",s.Angular_max_abs_for_LS)){ std::cout<<"\033[31mcant get param Angular_max_abs_for_LS\033[0m"<<std::endl;exit(0); }
    s.Angular_max_abs_for_HS = 0.087;    if(!nh.getParam(sNameSpace+"/"+"Angular_max_abs_for_HS",s.Angular_max_abs_for_HS)){ std::cout<<"\033[31mcant get param Angular_max_abs_for_HS\033[0m"<<std::endl;exit(0); }
    s.Angular_delta_max = 15*M_PI/180;    if(!nh.getParam(sNameSpace+"/"+"Angular_delta_max",s.Angular_delta_max)){ std::cout<<"\033[31mcant get param Angular_delta_max\033[0m"<<std::endl;exit(0); }
    s.Low_linear_for_HS_w_attenuate = 2.0;    if(!nh.getParam(sNameSpace+"/"+"Low_linear_for_HS_w_attenuate",s.Low_linear_for_HS_w_attenuate)){ std::cout<<"\033[31mcant get param Low_linear_for_HS_w_attenuate\033[0m"<<std::endl;exit(0); }
    s.High_linear_for_HS_w_attenuate = 8.0;    if(!nh.getParam(sNameSpace+"/"+"High_linear_for_HS_w_attenuate",s.High_linear_for_HS_w_attenuate)){ std::cout<<"\033[31mcant get param High_linear_for_HS_w_attenuate\033[0m"<<std::endl;exit(0); }

    s.Lon_P = 0.65;    if(!nh.getParam(sNameSpace+"/"+"Lon_P",s.Lon_P)){ std::cout<<"\033[31mcant get param Lon_P\033[0m"<<std::endl;exit(0); }
    s.Lon_I = 0.06;    if(!nh.getParam(sNameSpace+"/"+"Lon_I",s.Lon_I)){ std::cout<<"\033[31mcant get param Lon_I\033[0m"<<std::endl;exit(0); }
    s.Lon_D = 0.04;    if(!nh.getParam(sNameSpace+"/"+"Lon_D",s.Lon_D)){ std::cout<<"\033[31mcant get param Lon_D\033[0m"<<std::endl;exit(0); }
    s.Linear_max = 1.75;    if(!nh.getParam(sNameSpace+"/"+"Linear_max",s.Linear_max)){ std::cout<<"\033[31mcant get param Linear_max\033[0m"<<std::endl;exit(0); }
    s.Linear_min = 0;    if(!nh.getParam(sNameSpace+"/"+"Linear_min",s.Linear_min)){ std::cout<<"\033[31mcant get param Linear_min\033[0m"<<std::endl;exit(0); }
    s.Linear_delta_max = 0.25;    if(!nh.getParam(sNameSpace+"/"+"Linear_delta_max",s.Linear_delta_max)){ std::cout<<"\033[31mcant get param Linear_delta_max\033[0m"<<std::endl;exit(0); }

    s.DESIRED_LINEAR = 1.5;    if(!nh.getParam(sNameSpace+"/"+"DESIRED_LINEAR",s.DESIRED_LINEAR)){ std::cout<<"\033[31mcant get param DESIRED_LINEAR\033[0m"<<std::endl;exit(0); }
    s.DESIRED_MIN_LINEAR = 0.4;    if(!nh.getParam(sNameSpace+"/"+"DESIRED_MIN_LINEAR",s.DESIRED_MIN_LINEAR)){ std::cout<<"\033[31mcant get param DESIRED_MIN_LINEAR\033[0m"<<std::endl;exit(0); }
    s.d_near = 4;    if(!nh.getParam(sNameSpace+"/"+"d_near",s.d_near)){ std::cout<<"\033[31mcant get param d_near\033[0m"<<std::endl;exit(0); }
    s.d_stop = 999.99;    if(!nh.getParam(sNameSpace+"/"+"d_stop",s.d_stop)){ std::cout<<"\033[31mcant get param d_stop\033[0m"<<std::endl;exit(0); }
    s.k_ramp = 1;    if(!nh.getParam(sNameSpace+"/"+"k_ramp",s.k_ramp)){ std::cout<<"\033[31mcant get param k_ramp\033[0m"<<std::endl;exit(0); }
    s.w_threshold_sharp_turn = 1;    if(!nh.getParam(sNameSpace+"/"+"w_threshold_sharp_turn",s.w_threshold_sharp_turn)){ std::cout<<"\033[31mcant get param w_threshold_sharp_turn\033[0m"<<std::endl;exit(0); }
    s.w_threshold_gentle_turn = 2;    if(!nh.getParam(sNameSpace+"/"+"w_threshold_gentle_turn",s.w_threshold_gentle_turn)){ std::cout<<"\033[31mcant get param w_threshold_gentle_turn\033[0m"<<std::endl;exit(0); }

    s.Target_L = 0.25;    if(!nh.getParam(sNameSpace+"/"+"Target_L",s.Target_L)){ std::cout<<"\033[31mcant get param Target_L\033[0m"<<std::endl;exit(0); }
    s.Veh_L = 1;    if(!nh.getParam(sNameSpace+"/"+"Veh_L",s.Veh_L)){ std::cout<<"\033[31mcant get param Veh_L\033[0m"<<std::endl;exit(0); }
    s.PRE_DIS = 3.3;    if(!nh.getParam(sNameSpace+"/"+"PRE_DIS",s.PRE_DIS)){ std::cout<<"\033[31mcant get param PRE_DIS\033[0m"<<std::endl;exit(0); }
    s.u_weight = 0.3;    if(!nh.getParam(sNameSpace+"/"+"u_weight",s.u_weight)){ std::cout<<"\033[31mcant get param u_weight\033[0m"<<std::endl;exit(0); }


    // 参数提取成败判断
    if (s.d_stop == 999.99){ std::cout<<"\033[31mThe Params: You Dont Get Yaml Params! Check The sNameSpace or sth!!\033[0m"<<std::endl;exit(0); } 
    
    // 参数逻辑安全性检测
    if (s.Angular_max_abs_for_LS <= 0 || s.Angular_max_abs_for_HS<=0 || s.Angular_max_abs_for_HS>s.Angular_max_abs_for_LS){ std::cout<<"\033[31mThe Params: Angular_max_abs_for_LS or Angular_max_abs_for_HS set incurrectly\033[0m"<<std::endl;exit(0); }
    if (s.Low_linear_for_HS_w_attenuate <= 0 || s.High_linear_for_HS_w_attenuate<=0 || s.Low_linear_for_HS_w_attenuate>s.High_linear_for_HS_w_attenuate){ std::cout<<"\033[31mThe Params: Low_linear_for_HS_w_attenuate or High_linear_for_HS_w_attenuate set incurrectly\033[0m"<<std::endl;exit(0); }
    if (s.Linear_max <= s.DESIRED_LINEAR){ std::cout<<"\033[31mThe Params: Linear_max or DESIRED_LINEAR set incurrectly\033[0m"<<std::endl;exit(0); }
    if (s.DESIRED_LINEAR <= s.DESIRED_MIN_LINEAR){ std::cout<<"\033[31mThe Params: DESIRED_LINEAR or DESIRED_MIN_LINEAR set incurrectly\033[0m"<<std::endl;exit(0); }
    if (s.d_near <= s.d_stop){ std::cout<<"\033[31mThe Params: d_near or d_stop set incurrectly\033[0m"<<std::endl;exit(0); }
    if (s.w_threshold_gentle_turn >= s.w_threshold_sharp_turn){ std::cout<<"\033[31mThe Params: w_threshold_sharp_turn or w_threshold_gentle_turn set incurrectly\033[0m"<<std::endl;exit(0); }
    if (s.PRE_DIS <= 0){ std::cout<<"\033[31mThe Params: pre_dis set incurrectly\033[0m"<<std::endl;exit(0); }
}

void PathFollowOC::UpdataDynamicParams(DynamicParamsForPF &sFrom)
{
    UsingParams = sFrom;
}

void PathFollowOC::OdomCallback(const nav_msgs::Odometry& msg)//位姿回调函数，获取当前车辆位姿
{
    geometry_msgs::TransformStamped tfs;
    try
    {
        tfs = buffer_.lookupTransform("map", "veh", ros::Time(0));
        state_now.x = tfs.transform.translation.x;
        state_now.y = tfs.transform.translation.y;
        state_now.z = tfs.transform.translation.z; //得到NDT-GPS融合后的Z值，该Z与点云地图匹配，但不一定是真实物理世界（GPS海拔）的Z。
        state_now.yaw = tf2::getYaw(tfs.transform.rotation);
        state_now.linear = msg.twist.twist.linear.x;
    }
    catch(const tf2::TransformException& e)
    {
        ROS_ERROR_THROTTLE(1, "%s", e.what());
        return;
    }

    odom_flag = true;
}

void PathFollowOC::PathCallback(const nav_msgs::Path& msg)//路径回调函数，获取路径信息
{
    path_ref.resize(msg.poses.size());

    if (path_ref.size() == 0)
    {
        path_flag = false;
        // stop_flag = true;  
        doPivotSteer = true;
        ObsCtrlMode.data[0] = 1;
    }
    else
    {
        ROS_INFO_ONCE("[0]Now get the new path_ref !  Ready to AutoDrive ! ");
        path_flag = true;
        stop_flag = false;  // 如果解开注释：车辆stop后，将其开出停车范围，其可以自行继续自主行驶；     注释后：车辆stop后，永死，除非重启程序，更安全。
    }

    for (size_t i = 0; i < path_ref.size(); i++)
    {
        path_ref[i].x = msg.poses[i].pose.position.x;
        path_ref[i].y = msg.poses[i].pose.position.y;
    }
    
    for (size_t i = 0; i < path_ref.size(); i++)
    {
        if (i == 0)
        {
            path_ref[i].s = 0;
        }
        else
        {
            path_ref[i].s = path_ref[i-1].s + sqrt(pow((path_ref[i].y - path_ref[i-1].y), 2) + pow((path_ref[i].x - path_ref[i-1].x), 2));
        }
    }
    

}

void PathFollowOC::DesPointCallback(const geometry_msgs::PoseStampedConstPtr &msgPtr)
{
    // 引入终点的坐标，进行到达终点时的C级停车逻辑判断。（仅使用x y坐标，如果使用z，请载入PathFollow::OdomCallback函数中的state_now.z的值作为替代。）

    if (msgPtr->header.frame_id != "map"  &&  msgPtr->header.frame_id != "/map")
        ROS_INFO("\033[33mTracking suggest you pub Des pose in [map] frame by Rviz! \033[0m");

    // 将Rviz发布的初始位姿对齐到map坐标系下
    if (msgPtr->header.frame_id == "map" || msgPtr->header.frame_id == "/map") 
        des_pose_msg = *msgPtr;
    else
    {
        //get 从Rviz发布的Pose信息的坐标系 -> map坐标系 的tf
        geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
        get_transform("map", msgPtr->header.frame_id, TF_pose_to_map_ptr);

        // 将Rviz发布的终点Des的Pose信息变换至map坐标系
        geometry_msgs::PoseStamped::Ptr aftTF2Map_des_pose_msg_ptr(new geometry_msgs::PoseStamped);
        tf2::doTransform(*msgPtr, *aftTF2Map_des_pose_msg_ptr, *TF_pose_to_map_ptr);

        aftTF2Map_des_pose_msg_ptr->header.stamp = msgPtr->header.stamp;
        des_pose_msg = *aftTF2Map_des_pose_msg_ptr;
    }
    std::cout<<"已得到规划路径的终点: \n"<<des_pose_msg<<std::endl<<std::endl;;
    desPose_flag = true;
}

void PathFollowOC::SceneModeCallback(const sensor_msgs::BatteryStateConstPtr &msgPtr)
{
    static std::uint8_t last_mode = 0;

    k_p_linear_forXG = msgPtr->percentage;
    scene_mode_forXG = msgPtr->present;

    if (scene_mode_forXG == 0) // 低速模型
    {
        if (last_mode != 0)
            UpdataDynamicParams(LowSpeedParams);
    }
    else if(scene_mode_forXG == 1) // 高速模式
    {
        if (last_mode != 1)
        {
            UpdataDynamicParams(HighSpeedParams);
        }
        UsingParams.Lon_P = k_p_linear_forXG*HighSpeedParams.Lon_P ;
    }
    else // 错误
    {
        std::cout<<"\033[31mThe SceneMode: Error Mode!!\033[0m"<<std::endl;
        exit(0);
    }

    last_mode = scene_mode_forXG;
}


bool PathFollowOC::get_transform(const std::string & target_frame_id, const std::string & source_frame_id,
                                 const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
    std::string target_frame = remove_first_slash(target_frame_id);
    std::string source_frame = remove_first_slash(source_frame_id); 

    if (target_frame == source_frame) {
        transform_stamped_ptr->header.stamp = ros::Time::now();
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return true;
    }

    try {
        *transform_stamped_ptr = buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    } 
    catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

        transform_stamped_ptr->header.stamp = ros::Time::now();
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return false;
    }
    return true;
}

std::string PathFollowOC::remove_first_slash(std::string frame_id)
{
    const int slash_pos = frame_id.find('/');
    if(slash_pos == 0){
        frame_id.erase(0, 1);
    }
    return frame_id;
}