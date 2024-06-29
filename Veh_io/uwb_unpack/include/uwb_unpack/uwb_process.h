#pragma once
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <deque>

#include "common/utilities.h"

using namespace std;

class UWBProcess
{
public:
    UWBProcess(ros::NodeHandle & nh);

    bool uwbisok();
    void filter();
    void pubProcessResult();
    void clearCallbackFlag();

private:
    int size;
    ros::Subscriber uwbRawSub;
    ros::Publisher uwbProcessPub;
    float Tag2A0, Tag2A1;
    Point tag_point;
    deque<Point> PointsDeque;
    deque<Point> PointsDequeFilter;
    bool uwbFlag;

    void uwbRawCallback(const std_msgs::Float32MultiArray & msg);
    void locateTag(float Tag2A0, float Tag2A1, Point & tag);
    void updatePointsDeque(Point p);
    deque<Point> removeSharpPoint(deque<Point> input_deque);
    Point getProcessResult();
};