#include "uwb_follow/uwb_follow.h"

bool UWB::FollowStart()
{
    return follow_enable_flag;
}

bool UWB::CallbackFlag()
{
    currentTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastUpdateTime);
    if (elapsedTime >= filterDelay_Tag)
    {
        ROS_FATAL("CONNECTION RUNTIME OUT! please check connection.");

        cmd_msg.data[0] = 0;
        cmd_msg.data[1] = 0;
        PubCmd();
        brake_msg.data = true;
        PubBrake();

        UWB_flag = false ;
    }

    return UWB_flag;
}

void UWB::Stop()
{
    ROS_WARN_THROTTLE(1, "near target, dis = %f, stop and brake!", state_now.dis);

    state_now.linear = state_now.linear * Stop_linear_coefficient;
    state_now.angular = state_now.angular * Stop_angular_coefficient;

    if (state_now.linear < Stop_linear_threshold)
    {
        state_now.linear = 0;
        state_now.angular = 0;
        brake_msg.data = true;

        state_old = {};
        lat_error_deque.clear();
        lon_error_deque.clear();
    }
    else
    {
        brake_msg.data = false;
    }

    cmd_msg.data[0] = state_now.linear;
    cmd_msg.data[1] = state_now.angular;
}

void UWB::Rear()
{
    if (state_now.dis < Dis_rear && fabs(state_now.theta) < Rear_angle_limit)
    {
        ROS_WARN_THROTTLE(1, "too close to target!! dis = %f, rearing!", state_now.dis);
        state_now.linear = 0.5 * Linear_min;
        state_now.angular = 0;
        cmd_msg.data[0] = state_now.linear;
        cmd_msg.data[1] = 0;
        brake_msg.data = false;
    }
    else /*if (state_now.dis < Dis_brake)*/
    {
        ROS_WARN_THROTTLE(1, "near target, dis = %f, stop and brake!", state_now.dis);
        state_now.linear = 0;
        state_now.angular = 0;
        cmd_msg.data[0] = 0;
        cmd_msg.data[1] = 0;
        brake_msg.data = true;
    }

    state_old = {};
    lat_error_deque.clear();
    lon_error_deque.clear();
}

void UWB::GenLatCmd()
{    
    // 横向控制，本质是计算state_now.angular

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
    
    if (state_now.angular - state_old.angular > Angular_delta_max)
        state_now.angular = state_old.angular + Angular_delta_max;
    
    if (state_now.angular - state_old.angular < -Angular_delta_max)
        state_now.angular = state_old.angular - Angular_delta_max;

    if (state_now.angular > Angular_max)
        state_now.angular = Angular_max;
    
    if (state_now.angular < Angular_min)
        state_now.angular = Angular_min;

    if (fabs(state_now.angular) < 1e-2 && state_now.dis < 3.5)
        state_now.angular = 0;

    cmd_msg.data[1] = state_now.angular;
}

void UWB::GenLonCmd()
{   
    brake_msg.data = false;

    // 负角度时车辆停车，目前以80度以外就停车
    if (fabs(state_now.theta) > Angle_reverse)
    {
        Stop();
        return;
    }

    //三个临界点，Dis_rear倒车距离、Dis_brake刹车距离和Dis_stop停车距离，详细内容看readme
    //停车距离，进入该距离到刹车距离之间，减速停车
    //刹车距离，进入该距离直接刹车，包含减速停车来不及的情况和倒车倒多的情况
    //倒车距离，进入该距离后车辆将开始倒车，直至进入刹车距离，强制刹死
    if (state_now.dis < Dis_brake)   // 如果此帧距离目标距离小于倒车距离，以恒速度倒车
    {
        Rear();
        // Stop();  
        return;
    }
    else                            // 在正常前进区间和停车区间中都要进行pid控制
    {
        state_now.lon_error = state_now.dis - Target_L;

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


        if (state_now.dis < Dis_stop)// 距离目标距离小于停车距离，以平稳速度停车
        {
            Stop();
        }        
    }
    
    if (state_now.linear - state_old.linear > Linear_delta_max)
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