#pragma once

#include <iostream>
#include <string>  
#include <thread>

#include <ros/ros.h>
#include <tf/tf.h>

#include "can_unpack/socket_can.h"
#include "can_unpack/imu_gnss_can_manager.h"
#include "can_unpack/radar_can_manager.h"

class Can1Unpack
{
public:
    Can1Unpack(ros::NodeHandle & nh);
    ~Can1Unpack();

    void run();

private:
    // imu_gnss
    ImuGnss::ImuGnssManager imu_gnss_manager;

    // radar
    Radar::RadarCanManager radar_manager;

    // 多线程数据处理
    std::thread receive_thread;
    void receiveThread();

    // ros 部分，话题订阅和发布
    ros::NodeHandle nh_;
    std::string can_port_name;
};

