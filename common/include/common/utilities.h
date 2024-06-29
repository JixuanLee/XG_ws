#pragma once
#include <iostream>

struct Point
{
    float x;
    float y;
};

// 路径上的单个点
struct PathNode
{
    float x;
    float y;        //单位m
    float s;        //路径上的点距离初始路径点的距离，单位m
    float phi;      //路径上的点的切向角度，用该点的前后两点求出，单位rad
    // float c;        //路径上点的曲率，等于曲率半径的倒数，用该点和该点前一个点的距离s和转角求出。c = (phi - phi1) / (s - s1)
    // float u_ref;    //车辆到达该路径点时的参考车速 m/s
};

// 描述车辆当前点的运动信息状态
struct VehicleState
{
    float x;
    float y;                //在地图坐标系下的xy坐标，单位m
    float yaw;              //车辆横摆角，单位rad
    float linear;           //车辆的速速，单位m/s
    float angular;          //车辆点横摆角速度，单位rad/s

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

// 激光雷达采集到的点信息
class LaserPoint
{
public:
    float x_veh;        
    float y_veh;        //车辆坐标系下的采集点坐标
    float theta_veh;    //车辆坐标系下的采集点的偏角

    float dis_veh;      //采集点到车辆的距离
    float dis_tar;      //采集点到目标的距离，通常是标签
};

class LaserPoint_dis: public LaserPoint
{
public:
    float dis_cost;     //代价距离，为到车辆的距离和目标点的距离和

    bool operator<(const LaserPoint_dis & other) const       //在后续算法中，认为代价距离越大，排序的优先级越低
    {
        return dis_cost > other.dis_cost; // 小顶堆
    }
};