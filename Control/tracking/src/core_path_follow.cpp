#include "tracking/path_follow.h"

bool PathFollow::CallbackFlag(void)
{
    ROS_INFO_THROTTLE(1.5, "[0] odom_flag = %d ; path_flag = %d", odom_flag, path_flag);
    
    return odom_flag && path_flag;
}

void PathFollow::DynamicPredisAdjust(void)
{
    pre_dis = UsingParams.PRE_DIS + UsingParams.u_weight * state_now.linear;

    // 保证预瞄距离不会过于离谱，也排除底盘速度异常（例如过大的正负值速度等）带来的逻辑安全问题
    const float MAX_PRE_DIS = 9;
    if (pre_dis > MAX_PRE_DIS) 
        pre_dis = MAX_PRE_DIS;
    else if (pre_dis < UsingParams.PRE_DIS)
        pre_dis = UsingParams.PRE_DIS;
}

void PathFollow::FindTargetPoint(void)
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
void PathFollow::GenLonCmd(void)
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
void PathFollow::GenLatCmd(void)
{
    // 1.动态期望速度方案：使用  [动态自适应期望线速度]  作为实际下发角速度
    // GetLatDesiredAngular();
    // cmd.angular.z = state_now.desired_angular_now;

    // 2.PID距离夹角方案：使用  [目标与当前车辆位姿的夹角]  作为PID输入，控制实际下发角速度
    cmd.angular.z = LatPID(state_now.theta);

}


float PathFollow::LonPID(double error)
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

float PathFollow::LatPID(double error)
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

void PathFollow::UpdataVehicleStateData(void)
{
    // 该函数单独写出来的目的是，从之前“在某一PID函数（取决于Lat和Lon的生成顺序，在后生成者的PID函数中）的最后一步进行”
    // 转移至单独的解耦函数，防止Lat和Lon的生成顺序对State的更新产生逻辑影响。
    state_old = state_now; // 更新state
}

void PathFollow::SteeringLinearAttenuation(void)
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

void PathFollow::StartingLinearAmplification(void)
{
    // Debug记录-0501：XG：①可能底盘自带起步逻辑②简单测试后没有太大效果，可能是底盘反馈速度有问题？或者代码逻辑不完善？
    // 在 2.PID距离夹角方案 中，在PID输出线速度与角速度后，进行二次优化，改善起步工况的平顺性。
    float delta_linear = cmd.linear.x - state_now.linear; // 预下发速度 - 当前车速

    if (delta_linear > 0) // 只针对加速情况进行优化（减速情况PID内含，无需优化）
    {
        if (delta_linear > UsingParams.Linear_delta_max)
        {
            cmd.linear.x = state_now.linear + UsingParams.Linear_delta_max;
        }
    }

    // 安全冗余措施 -- 不要轻易删除，以免不经意的调参/改动 引发行车安全问题！
    if (cmd.linear.x > UsingParams.Linear_max)
        cmd.linear.x = UsingParams.Linear_max;
    if (cmd.linear.x < UsingParams.Linear_min)
        cmd.linear.x = UsingParams.Linear_min;
}

void PathFollow::HighSpeedAngularMaxAttenuation(void)
{
    // 针对高速工况，进行角速度二次限制，保证行车安全。
    float Angular_max_abs_Realistic;

    float linear_abs = std::abs(cmd.linear.x);

    // 线速度比较小，则按照给定的低速工况最大角速度（数值大）进行限制
    if (linear_abs <= UsingParams.Low_linear_for_HS_w_attenuate)
    {
        Angular_max_abs_Realistic = UsingParams.Angular_max_abs_for_LS;
    }
    // 线速度不大不小，则按照线性原则进行最大角速度（数值线性衰减）限制
    else if (linear_abs > UsingParams.Low_linear_for_HS_w_attenuate && linear_abs < UsingParams.High_linear_for_HS_w_attenuate)
    {
        // wmax = k*v+b, k=(wmax_low-wmax_high)/(vlow-vhigh), b=(wmax_high*vlow-wmax_low*vhigh)/(vlow-vhigh)
        Angular_max_abs_Realistic =  (UsingParams.Angular_max_abs_for_LS-UsingParams.Angular_max_abs_for_HS)/(UsingParams.Low_linear_for_HS_w_attenuate-UsingParams.High_linear_for_HS_w_attenuate) * linear_abs 
                                                                    + (UsingParams.Angular_max_abs_for_HS*UsingParams.Low_linear_for_HS_w_attenuate-UsingParams.Angular_max_abs_for_LS*UsingParams.High_linear_for_HS_w_attenuate)/(UsingParams.Low_linear_for_HS_w_attenuate-UsingParams.High_linear_for_HS_w_attenuate);
    }
    // 线速度很大，则按照给定的高速工况最大角速度（数值小）进行限制
    else
    {
        Angular_max_abs_Realistic = UsingParams.Angular_max_abs_for_HS;
    }

    // 安全冗余措施 -- 不要轻易删除，以免不经意的调参/改动 引发行车安全问题！
    if (Angular_max_abs_Realistic > UsingParams.Angular_max_abs_for_LS)
        Angular_max_abs_Realistic = UsingParams.Angular_max_abs_for_LS;
    else if (Angular_max_abs_Realistic < UsingParams.Angular_max_abs_for_HS)
        Angular_max_abs_Realistic = UsingParams.Angular_max_abs_for_HS;

    // 具体的角速度二次限制
    if (cmd.angular.z > Angular_max_abs_Realistic)
        cmd.angular.z = Angular_max_abs_Realistic;
    if (cmd.angular.z < -Angular_max_abs_Realistic)
        cmd.angular.z = -Angular_max_abs_Realistic;

}

void PathFollow::GetLonDesiredLinear(void)
{
    // XG：2024-05尚未使用。
    std::cout << "Have get the DESIRED_LINEAR: "<< UsingParams.DESIRED_LINEAR<<std::endl;

    // 目前各个环节的期望速度为 串联 
    float desired_steer_linear = 0;
    float desired_ramp_linear = 0;
    float desired_near_linear = 0;

    // 1.转弯工况：减速
    {
        //说明：以急弯阈值r_threshold_sharp_turn为1，缓弯阈值r_threshold_gentle_turn为2为例：↓
        //如果r<=1，认为是急弯，应保持最基本的速度，设置为最小巡航速度DESIRED_MIN_LINEAR；
        //如果r在(1,2)，认为是正常弯道，进行期望速度线性衰减；
        //如果r>=2，认为是小弯/直线巡航，期望速度不变，为DESIRED_LINEAR。
        const static float r_threshold_sharp_turn = 1;
        const static float r_threshold_gentle_turn = 2;

        float pre_r_abs = std::abs(pre_r);
        // 急弯，低速
        if (pre_r_abs <= r_threshold_sharp_turn)
        {
            desired_steer_linear = UsingParams.DESIRED_MIN_LINEAR;
        }
        // 一般弯，线性减速
        else if (pre_r_abs>r_threshold_sharp_turn && pre_r_abs<r_threshold_gentle_turn)
        {
            // v = kr + b;    k = (vd-vmin)/(rg-rs), b = (vmin*rg - vd*rs)/(rg-rs)
            desired_steer_linear = ((UsingParams.DESIRED_LINEAR-UsingParams.DESIRED_MIN_LINEAR)/(r_threshold_gentle_turn-r_threshold_sharp_turn)) * pre_r  +  ((UsingParams.DESIRED_MIN_LINEAR*r_threshold_gentle_turn-UsingParams.DESIRED_LINEAR*r_threshold_sharp_turn)/(r_threshold_gentle_turn-r_threshold_sharp_turn));
        }
        // 缓弯or直线，高速
        else
            desired_steer_linear = UsingParams.DESIRED_LINEAR;


    }

    // 2.坡道工况：加速/减速
    {
        // TODO:完善逻辑
        desired_ramp_linear = UsingParams.DESIRED_LINEAR;
    }

    // 3.近距工况：减速
    {
        // 远距离，高速巡航
        if (state_now.dis >= UsingParams.d_near)
            desired_near_linear = UsingParams.DESIRED_LINEAR;
        // 中距离，线性减速
        else if (state_now.dis < UsingParams.d_near && state_now.dis > UsingParams.d_stop)
        {
            // v = kd + b;  k = (vd-vmin)/(dn-ds)  b = (vmin*dn - vd*ds)/(dn-ds)
            desired_near_linear = (UsingParams.DESIRED_LINEAR - UsingParams.DESIRED_MIN_LINEAR)/(UsingParams.d_near - UsingParams.d_stop)*state_now.dis + (UsingParams.DESIRED_MIN_LINEAR*UsingParams.d_near - UsingParams.DESIRED_LINEAR*UsingParams.d_stop)/(UsingParams.d_near-UsingParams.d_stop);
        }
        //近距离，刹停！
        else 
        {
            desired_near_linear = 0;
            Stop();   
        }
    }
    
    state_now.desired_linear_now = UsingParams.DESIRED_LINEAR*desired_steer_linear/UsingParams.DESIRED_LINEAR*desired_ramp_linear/UsingParams.DESIRED_LINEAR*desired_near_linear/UsingParams.DESIRED_LINEAR;
    
    // 防止速度超出阈值
    if (state_now.desired_linear_now > UsingParams.Linear_max)
        state_now.desired_linear_now = UsingParams.Linear_max;
    if (state_now.desired_linear_now < UsingParams.Linear_min)
        state_now.desired_linear_now = UsingParams.Linear_min;
}

void PathFollow::GetLatDesiredAngular(void)
{
    // XG：2024-05尚未使用。
    //根据XG经验，按照数学逻辑得到的期望w往往在底盘执行的并不优秀，需要魔法参数进行线性比例调节
    double k_magic = 1.0; 

    //1.使用 [动态期望v] 和 [单点预瞄的pre_r] ：w=v/r
    state_now.desired_angular_now = state_now.desired_linear_now / pre_r * k_magic;
    //2.使用 [车辆实际v] 和 [单点预瞄的pre_r] ：w=v/r
    // state_now.desired_angular_now = state_now.linear / pre_r * k_magic;
}

void PathFollow::Stop()
{
    std::cout << "\033[33m[warn] Near Target or Errors Occured!\033[0m]" << std::endl;
    stop_flag = true;
    path_flag = false;
}