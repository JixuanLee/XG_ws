#include "tracking_obs_cross/path_follow_oc.h"

bool PathFollowOC::CallbackFlag(void)
{
    ROS_INFO_THROTTLE(1.5, "[0] odom_flag = %d ; path_flag = %d", odom_flag, path_flag);
    
    return odom_flag && path_flag;
}

void PathFollowOC::DynamicPredisAdjust(void)
{
    pre_dis = UsingParams.PRE_DIS + UsingParams.u_weight * state_now.linear;

    // 保证预瞄距离不会过于离谱，也排除底盘速度异常（例如过大的正负值速度等）带来的逻辑安全问题
    const float MAX_PRE_DIS = 9;
    if (pre_dis > MAX_PRE_DIS) 
        pre_dis = MAX_PRE_DIS;
    else if (pre_dis < UsingParams.PRE_DIS)
        pre_dis = UsingParams.PRE_DIS;
}

void PathFollowOC::FindTargetPoint(void)
{
    // 找最近路点
    float dis_min = 9999; // 初始化足够大即可
    int near_num = -1;   // 最近点下标

    for (size_t i = 0; i < path_ref.size(); i++)
    {
        double dis_tmp;
        if (isMapPath_flag)
            dis_tmp = sqrt(pow(path_ref[i].x - state_now.x, 2) + pow(path_ref[i].y - state_now.y , 2));  // 采用map下的路径时，需要此
        else
            dis_tmp = sqrt(pow(path_ref[i].x, 2) + pow(path_ref[i].y, 2));

        if (dis_tmp < dis_min)
        {
            dis_min = dis_tmp;
            near_num = i;
        }
    }

    if(near_num < 0)
    {
        std::cout << "\033[31m[1] Can't Find Nearest Point !!\033[0m" << std::endl;
        Stop();
    }
    else
    ROS_INFO_THROTTLE(0.25, "[1] The Nearest Point is No.%d", near_num);


    // 找目标路点
    aim_num = -1;
    static bool isDestinationPoint = false;
    if (dis_min > pre_dis) //车离轨迹很远，则优先往最近点靠
    {
        int front_bias_points_num = 15; // 为了保证在 [车辆远离局部轨迹线] 时，车辆能往正确的方向控制，进行前向偏差矫正
        aim_num = std::min(near_num + front_bias_points_num, int(path_ref.size()-1));
    }
    else
    {
        for (size_t i = near_num; i < path_ref.size(); i++)
        {
            if (path_ref[i].s - path_ref[near_num].s > pre_dis)//[路点] 距离 [最近路点] 的距离大于预瞄距离，将该点设置为目标点
            {
                aim_num = i;
                break;
            }
        }
        //若遍历到最终点，将终点设置为目标点
        if (aim_num == -1)
        {
            aim_num = path_ref.size()-1;
            isDestinationPoint = true;
        }
    }

    if (isDestinationPoint)
        ROS_INFO_THROTTLE(0.25, "[2] The Aim Point is No.%d, It's Destination Point", aim_num);
    else
        ROS_INFO_THROTTLE(0.25, "[2] The Aim Point is No.%d", aim_num);

    isDestinationPoint = false;


    // 目标点与车辆自身的  距离  夹角  转弯半径（估计）
    if (isMapPath_flag)// 采用map下的路径时，需要此
    {
        aim_dis = sqrt(pow(path_ref[aim_num].x - state_now.x, 2) + pow(path_ref[aim_num].y - state_now.y, 2));  
        geometry_msgs::PointStamped aim_point_in_map;  
        geometry_msgs::PointStamped aim_point_in_veh;
        aim_point_in_map.header.stamp = ros::Time();
        aim_point_in_map.header.frame_id = "map";
        aim_point_in_map.point.x = path_ref[aim_num].x;
        aim_point_in_map.point.y = path_ref[aim_num].y;
        aim_point_in_map.point.z = state_now.z;
        // ROS_INFO_THROTTLE(0.25, "[3] Aim's  y---x-inMap: %.3f , %.3f", aim_point_in_map.point.y, aim_point_in_map.point.x);

        try{
            aim_point_in_veh = buffer_.transform(aim_point_in_map,"veh");
        }
        catch(const std::exception& e){
            ROS_INFO("TF ERROR:%s",e.what());
            Stop();
        }
        pre_angle = atan2(aim_point_in_veh.point.y, aim_point_in_veh.point.x);
        ROS_INFO_THROTTLE(0.25, "[3] Aim's  y---x-inVeh: %.3f , %.3f", aim_point_in_veh.point.y, aim_point_in_veh.point.x);
    }
    else
    {
        aim_dis = sqrt(pow(path_ref[aim_num].x , 2) + pow(path_ref[aim_num].y, 2)); 
        pre_angle = atan2(path_ref[aim_num].y, path_ref[aim_num].x);
        ROS_INFO_THROTTLE(0.25, "[3] Aim's  y---x-inVeh: %.3f , %.3f", path_ref[aim_num].y, path_ref[aim_num].x);
    }

    pre_r = aim_dis/2/sin(pre_angle);

    state_now.dis = aim_dis;
    state_now.theta = pre_angle;

}


//纵向控制
void PathFollowOC::GenLonCmd(void)
{   
    // Tips: 停车安全逻辑：（均在纵向控制Lon实现）
    // A.首先对车辆实际位置安全性进行考察（包括到局部路径的aim点、到全局路径的des点），如果有危险则直接刹车
    // B.接着对车辆在轨迹上的 near点 和 aim点 之间的距离进行考察，在 [动态期望速度方案] 中，在GetLonDesiredLinear()实现；在 [PID距离夹角方案] 中，在 其下的if实现。

    // A级停车逻辑
    {
        double veh2aim_dis, veh2des_dis;
        if (isMapPath_flag)
        {
            veh2aim_dis = sqrt((state_now.x-path_ref[aim_num].x)*(state_now.x-path_ref[aim_num].x) + (state_now.y-path_ref[aim_num].y)*(state_now.y-path_ref[aim_num].y)  ) ;// 采用map下的路径时，需要此
            veh2des_dis = sqrt((state_now.x-des_pose_msg.pose.position.x)*(state_now.x-des_pose_msg.pose.position.x) + (state_now.y-des_pose_msg.pose.position.y)*(state_now.y-des_pose_msg.pose.position.y)  ) ;// 采用map下的路径时，需要此
        }        
        else
        {
            veh2aim_dis = sqrt((path_ref[aim_num].x)*(path_ref[aim_num].x) + (path_ref[aim_num].y)*(path_ref[aim_num].y)  ) ;
            veh2des_dis = sqrt((des_pose_msg.pose.position.x)*(des_pose_msg.pose.position.x) + (des_pose_msg.pose.position.y)*(des_pose_msg.pose.position.y)  ) ;
        }

        if (veh2aim_dis < UsingParams.d_stop)
        {
            std::cout << "\033[33m[warn] The Veh Real Position Is Too Close To The [AimPoint] !!\033[0m" << std::endl;
            Stop();
        }
        else if (veh2des_dis < UsingParams.d_stop && desPose_flag) // 当订阅到des点才进行该检测
        {
            std::cout << "\033[33m[warn] The Veh Real Position Is Too Close To The [DesPoint] !!\033[0m" << std::endl;
            desPose_flag = false; // 进入该elas if，说明到达了上次发布的终点，重置flag，等待下次发布规划命令。
            Stop();
        }
        else
        {
            stop_flag = false;
        }
    }

    // B级停车逻辑 - - - - - 亦为纵向控制核心算法
    {
        // 1.动态期望速度方案：使用  [动态自适应期望线速度]  作为实际下发速度
        // GetLonDesiredLinear(); //内含B级停车逻辑
        // cmd.linear.x = state_now.desired_linear_now;

        // 2.PID距离夹角方案：使用  [目标距离]  作为PID输入，控制实际下发速度
        if (state_now.dis <= UsingParams.d_stop) 
        {
            Stop();
        }
        else
        {
            cmd.linear.x = LonPID(state_now.dis - UsingParams.Target_L); 
            stop_flag = false;
        }
    }

}

// 横向控制
void PathFollowOC::GenLatCmd(void)
{
    // 1.动态期望速度方案：使用  [动态自适应期望线速度]  作为实际下发角速度
    // GetLatDesiredAngular();
    // cmd.angular.z = state_now.desired_angular_now;

    // 2.PID距离夹角方案：使用  [目标与当前车辆位姿的夹角]  作为PID输入，控制实际下发角速度
    cmd.angular.z = LatPID(state_now.theta);

}


float PathFollowOC::LonPID(double error)
{
    state_now.lon_error = error;

    if ((state_old.linear < UsingParams.Linear_max && state_old.linear > UsingParams.Linear_min) )
    {
        lon_error_deque.push_back(state_now.lon_error);
    }

    if (lon_error_deque.size() > Error_deque_size)
    {
        lon_error_deque.pop_front();
    }

    state_now.lon_error_sum = 0;
    for (uint i = 0; i < lon_error_deque.size(); i++)
    {
        state_now.lon_error_sum += lon_error_deque[i];
    }

    state_now.lon_error_diff = (state_now.lon_error - state_old.lon_error) * RATE;

    //PID计算
    state_now.linear = UsingParams.Lon_P * state_now.lon_error + 
                        UsingParams.Lon_I * state_now.lon_error_sum / RATE +
                        UsingParams.Lon_D * state_now.lon_error_diff;
    
    // 防止速度梯度过大
    if (state_now.linear - state_old.linear > UsingParams.Linear_delta_max)
        state_now.linear = state_old.linear + UsingParams.Linear_delta_max;
    if (state_now.linear - state_old.linear < -UsingParams.Linear_delta_max)
        state_now.linear = state_old.linear - UsingParams.Linear_delta_max;
    
    // 防止速度超出阈值
    if (state_now.linear > UsingParams.Linear_max)
        state_now.linear = UsingParams.Linear_max;
    if (state_now.linear < UsingParams.Linear_min)
        state_now.linear = UsingParams.Linear_min;
    
    return state_now.linear;
}

float PathFollowOC::LatPID(double error)
{
    state_now.lat_error = error;

    lat_error_deque.push_back(state_now.lat_error);
    if (lat_error_deque.size() > Error_deque_size)  
    {
        lat_error_deque.pop_front();
    }

    state_now.lat_error_sum = 0;
    for (uint i = 0; i < lat_error_deque.size(); i++)
    {
        state_now.lat_error_sum += lat_error_deque[i];
    }

    state_now.lat_error_diff = (state_now.lat_error - state_old.lat_error) * RATE;

    // PID计算
    state_now.angular = UsingParams.Lat_P * state_now.lat_error +
                        UsingParams.Lat_I * state_now.lat_error_sum / RATE +
                        UsingParams.Lat_D * state_now.lat_error_diff;
    
    // 防止速度梯度过大
    if (state_now.angular - state_old.angular > UsingParams.Angular_delta_max)
        state_now.angular = state_old.angular + UsingParams.Angular_delta_max;
    if (state_now.angular - state_old.angular < -UsingParams.Angular_delta_max)
        state_now.angular = state_old.angular - UsingParams.Angular_delta_max;

    // 防止速度超出阈值
    if (state_now.angular > UsingParams.Angular_max_abs_for_LS)
        state_now.angular = UsingParams.Angular_max_abs_for_LS;
    if (state_now.angular < -UsingParams.Angular_max_abs_for_LS)
        state_now.angular = -UsingParams.Angular_max_abs_for_LS;
        
    // 防止抖动。但在距离目标点比较近时，放宽条件，允许小抖动（方便调整最终位姿）//但是徐工底盘的分辨率：0.1 rad/s，所以没实际用处，留模板给后面项目
    if (fabs(state_now.angular) < 0.0035 || 
        (fabs(state_now.angular) < 0.02 && state_now.dis < 3.5)     )  
        state_now.angular = 0;

    return state_now.angular;
}

void PathFollowOC::UpdataVehicleStateData(void)
{
    // 该函数单独写出来的目的是，从之前“在某一PID函数（取决于Lat和Lon的生成顺序，在后生成者的PID函数中）的最后一步进行”
    // 转移至单独的解耦函数，防止Lat和Lon的生成顺序对State的更新产生逻辑影响。
    state_old = state_now; // 更新state
}

void PathFollowOC::SteeringLinearAttenuation(void)
{
    // 在 2.PID距离夹角方案 中，在PID输出线速度与角速度后，进行二次优化，改善转向工况的平顺性。
    float w_abs = std::abs(cmd.angular.z);
    // 只在原始速度较高时，考虑是否衰减
    if (cmd.linear.x > UsingParams.DESIRED_MIN_LINEAR)
    {
        // 急弯，最低速
        if (w_abs >= UsingParams.w_threshold_sharp_turn)
        {
            cmd.linear.x = UsingParams.DESIRED_MIN_LINEAR;
        }
        // 一般弯，线性减速
        else if (w_abs<UsingParams.w_threshold_sharp_turn && w_abs>UsingParams.w_threshold_gentle_turn)
        {   
            // v = kw + b;    k = (v-vmin)/(wg-ws), b = (vmin*wg - v*ws)/(wg-ws)
            cmd.linear.x = ((cmd.linear.x-UsingParams.DESIRED_MIN_LINEAR)/(UsingParams.w_threshold_gentle_turn-UsingParams.w_threshold_sharp_turn)) * w_abs  +  ((UsingParams.DESIRED_MIN_LINEAR*UsingParams.w_threshold_gentle_turn-cmd.linear.x*UsingParams.w_threshold_sharp_turn)/(UsingParams.w_threshold_gentle_turn-UsingParams.w_threshold_sharp_turn));
        }
        // 缓弯or直线，保持高速不变
    }

    // 安全冗余措施 -- 不要轻易删除，以免不经意的调参/改动 引发行车安全问题！
    if (cmd.linear.x > UsingParams.Linear_max)
        cmd.linear.x = UsingParams.Linear_max;
    if (cmd.linear.x < UsingParams.Linear_min)
        cmd.linear.x = UsingParams.Linear_min;

}

void PathFollowOC::PivotSteering(void)
{
    static const float halfIntervalOfAngle = 1/180*M_PI;
    float nowAngleLow = aimPivotSteerAngle-halfIntervalOfAngle;
    float nowAngleHigh = aimPivotSteerAngle+halfIntervalOfAngle;

    // 使用 fmod 函数将角度限制在 0 到 2π 之间
    nowAngleLow = std::fmod(nowAngleLow, 2.0 * M_PI);
    nowAngleHigh = std::fmod(nowAngleHigh, 2.0 * M_PI);

    // 如果角度在 -π 到 π 之间
    if (nowAngleLow > M_PI) {
        nowAngleLow -= 2.0 * M_PI;
    }
    if (nowAngleHigh > M_PI) {
        nowAngleHigh -= 2.0 * M_PI;
    }
    if (nowAngleLow < -M_PI) {
        nowAngleLow += 2.0 * M_PI;
    } 
    if (nowAngleHigh < -M_PI) {
        nowAngleHigh += 2.0 * M_PI;
    } 


    if (state_now.yaw>=nowAngleLow && state_now.yaw<=nowAngleHigh)
    {
        ROS_INFO_ONCE("[2]Now finish pivot steer ! ");
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        brake.data = false;
        doPivotSteer = false;
        ObsCtrlMode.data[0] = 0;
        doObsCrossProcess = true;
        PubCmd();
        sleep(2); // 原地转向完歇一会
    }
    else
    {
        ROS_INFO_ONCE("[1]Now start pivot steer ! ");
        cmd.linear.x = 0;
        cmd.angular.z = -0.3; // 可优化
        brake.data = false;
        PubCmd();
    }
}

void PathFollowOC::ObsCrossProcess(void)
{
    // 先倒退xx米
    static float nowDisToMove = 0.0;

    if (nowDisToMove < aimDisToMove)
    {
        ROS_INFO_ONCE("[3]Now start move backforward ! ");
        cmd.linear.x = -0.2;
        cmd.angular.z = 0;
        brake.data = false;
        PubCmd();
        nowDisToMove += std::abs(cmd.linear.x)/RATE;
    }
    else
    {
        ROS_INFO_ONCE("[4]Now finish move backforward ! ");
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        brake.data = false;

        ROS_INFO_ONCE("[5]Now start obs cross ! ");
        ObsCtrlMode.data[1] = 1;
        PubCmd();
    }
    



    // 如果最后底盘可以反馈“越障结束”，则TODO: 恢复doObsCrossProcess = false;
    // {
    //     ...
    //     ROS_INFO_ONCE("[6]Now finish obs cross ! ");
    //     ObsCtrlMode.data[1] = 0;
    //     doObsCrossProcess = false;
    //     cmd.linear.x = 0;
    //     cmd.angular.z = 0;
    //     brake.data = false;
    //     PubCmd();
    //     ROS_INFO_ONCE("[-1]Now waiting for new path_ref ... ");
    //     ...
    // }
    
}

void PathFollowOC::Stop()
{
    std::cout << "\033[33m[warn] Near Target or Errors Occured!\033[0m]" << std::endl;
    stop_flag = true;
    path_flag = false;
}