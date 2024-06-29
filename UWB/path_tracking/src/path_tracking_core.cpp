#include "path_tracking/path_tracking.h"

bool PathTracking::CallbackFlag()
{
    now_counter++;
    if (now_counter >= max_counter)
        {
            ROS_ERROR("CONNECTION RUNTIME OUT! please check connection.");

            cmd_msg.data[0] = 0;
            cmd_msg.data[1] = 0;
            PubCmd();
            brake_msg.data = true;
            PubBrake();

            exit(-1);
        }
        
    return odom_flag && chassis_flag && path_flag;
}

void PathTracking::ClearCallbackFlag()
{
    odom_flag = false;
    chassis_flag = false;
    path_flag = false;
    now_counter = 0;
}

void PathTracking::Stop(void)
{
    ROS_INFO("near destination, stop and brake!");

    cmd_msg.data[0] = 0;
    cmd_msg.data[1] = 0;
    brake_msg.data = true;
}

void PathTracking::PushPathNode()
{
    // 找最近路点
    dis_min = 1000; // 初始化足够大即可
    near_num = 0;

    for (uint i = 0; i < path_ref.size(); i++)
    {
        dis_tmp = sqrt(pow(path_ref[i].x - state_now.x, 2) + pow(path_ref[i].y - state_now.y, 2));  
        if (dis_tmp < dis_min)
        {
            dis_min = dis_tmp;
            near_num = i;
        }
    }

    // 去掉车后经过的点
    path_ref.erase(path_ref.begin(), path_ref.begin() + near_num);
}

void PathTracking::GenLatCmd()
{
    pre_dis = pre_dis_min + U_weight * fabs(state_now.angular);
    std::cout << "pre_dis = " << pre_dis << std::endl;

    // 最近路点目前已经变为0
    near_num = 0;

    // 找目标路点
    if (dis_min > pre_dis)
    {
        aim_num = near_num;
    }
    else
    {
        for (uint i = 0; i < path_ref.size(); i++)
        {
            if (path_ref[i].s - path_ref[near_num].s > pre_dis)
            {
                aim_num = i;
                break;
            }
            
            if (i == path_ref.size() - 1)
            {
                aim_num = path_ref.size() - 1;
            }
        }
    }

    // 在接近路径终点处停车
    if ((aim_num == path_ref.size() - 1) &&                         //瞄到了终点
        (path_ref[aim_num].s - path_ref[near_num].s < Near_dis))    //且很靠近终点
    {
        Stop();
        return;            
    }

    // 单点预瞄
    aim_dis = sqrt(pow(path_ref[aim_num].x - state_now.x, 2) + pow(path_ref[aim_num].y - state_now.y, 2));
    aim_angle = atan2(path_ref[aim_num].y - state_now.y, path_ref[aim_num].x - state_now.x) - state_now.yaw;
    state_now.dis = aim_dis;
    state_now.theta = aim_angle;

    // pid控制
    state_now.lat_error = state_now.theta;
    lat_error_deque.push_back(state_now.lat_error);
    if (lat_error_deque.size() > Error_deque_size)  // Error_deque_size = 3*RATE
    {
        lat_error_deque.pop_front();
    }

    state_now.lat_error_sum = 0;
    for (uint i = 0; i < lat_error_deque.size(); i++)
    {
        state_now.lat_error_sum += lat_error_deque[i];
    }

    state_now.lat_error_diff = (state_now.lat_error - state_old.lat_error) * RATE;

    state_now.angular = Lat_P * state_now.lat_error +
                        Lat_I * state_now.lat_error_sum / RATE +
                        Lat_D * state_now.lat_error_diff;    
    
    if (state_now.angular > Angular_max)
        state_now.angular = Angular_max;
    
    if (state_now.angular < Angular_min)
        state_now.angular = Angular_min;

    if (fabs(state_now.angular) < 1e-2)
        state_now.angular = 0;
    
    cmd_msg.data[1] = state_now.angular;
}

void PathTracking::GenLonCmd()
{
    // 起步    
    // if (counter < 100)
    // {
    //     cmd_msg.data[0] = Torque_start;
    //     counter++;
    //     return;
    // }

    state_now.lon_error = state_now.dis;

    if ((state_old.linear < Linear_max && state_old.linear > Linear_min) ||
        (state_old.linear == Linear_max && state_now.lon_error < 0) ||
        (state_old.linear == Linear_min && state_now.lon_error > 0))
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
    
    state_now.linear = Lon_P * state_now.lon_error + 
                       Lon_I * state_now.lon_error_sum / RATE +
                       Lon_D * state_now.lon_error_diff;
    
    if (state_now.linear - state_old.linear > Linear_delta_max)  // 限制最大加速度ljx
        state_now.linear = state_old.linear + Linear_delta_max;
    
    if (state_now.linear - state_old.linear < -Linear_delta_max)
        state_now.linear = state_old.linear - Linear_delta_max;
    
    if (state_now.linear > Linear_max)
        state_now.linear = Linear_max;
    
    if (state_now.linear < Linear_min)
        state_now.linear = Linear_min;
    
    cmd_msg.data[0] = state_now.linear;
    state_old = state_now;
}

