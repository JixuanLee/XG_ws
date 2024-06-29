// 此文件用于接收UWB解包后已经进行过定位计算的定位数据，并对定位数据/uwb_raw进行处理，并发布为/uwb话题
#include "uwb_unpack/uwb_process.h"
const float gap = 1.6;


UWBProcess::UWBProcess(ros::NodeHandle & nh)
{
    uwbRawSub = nh.subscribe("/uwb_raw", 10, &UWBProcess::uwbRawCallback, this); // 取数据存入PointsDeque
    uwbProcessPub = nh.advertise<geometry_msgs::PointStamped>("/uwb_process", 10);
    size = 20;
    uwbFlag = false;
}

void UWBProcess::uwbRawCallback(const std_msgs::Float32MultiArray & msg) 
{
    if (msg.data.size() < 2)
    {
        ROS_ERROR("msg /uwb_raw is wrong!");
        return;
    }

    Tag2A0 = msg.data[0];
    Tag2A1 = msg.data[1];
    if ((Tag2A0 + Tag2A1 >= gap) &&
        (Tag2A0 + gap >= Tag2A1) &&
        (Tag2A1 + gap >= Tag2A0))
    {
        locateTag(Tag2A0, Tag2A1, tag_point);
        updatePointsDeque(tag_point);
        uwbFlag = true;
    }
}

void UWBProcess::clearCallbackFlag()
{
    uwbFlag = false;
}

void UWBProcess::locateTag(float Tag2A0, float Tag2A1, Point & tag)
{
    tag.y = (Tag2A0*Tag2A0 - Tag2A1*Tag2A1 + gap*gap) / (2 * gap);
    tag.x = sqrt(Tag2A0*Tag2A0 - tag.y*tag.y);

    // 自己解算uwb坐标时，将原始坐标进行偏移
    tag.y -= gap / 2; //使得x轴改为车辆纵向中轴线
}

void UWBProcess::updatePointsDeque(Point p) 
{
    PointsDeque.push_back(p);
    if( PointsDeque.size() > size)
        PointsDeque.pop_front();
}

bool UWBProcess::uwbisok() 
{
    return uwbFlag;
}

void UWBProcess::filter() 
{
    PointsDequeFilter = removeSharpPoint(PointsDeque);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
    PointsDequeFilter = removeSharpPoint(PointsDequeFilter);
}

deque<Point> UWBProcess::removeSharpPoint(deque<Point> input_deque) 
{
    deque<Point> output_deque;
    double sum = 0;
    double average = 0;
    double error;
    double error_max;
    int max_x;
    int max_y;

    for(int i = 0; i<input_deque.size(); ++i) 
        sum += input_deque[i].x;

    average = sum / input_deque.size();

    for(int i = 0; i<input_deque.size(); ++i) 
    {
        error = fabs(input_deque[i].x - average);
        if(error > error_max) 
        {
            error_max = error;
            max_x = i;
        }
    }

    sum = 0;
    error_max = 0;

    for(int i = 0; i<input_deque.size(); ++i) 
        sum += input_deque[i].y;

    average = sum / input_deque.size();

    for(int i = 0; i<input_deque.size(); ++i) 
    {
        error = fabs( input_deque[i].y - average);
        if(error > error_max) 
        {
            error_max = error;
            max_y = i;
        }
    }

    output_deque.clear();

    for(int i = 0; i < input_deque.size(); ++i) 
    {
        if(i == max_x || i == max_y)
            continue;

        else
            output_deque.push_back(input_deque[i]);
    }

    return output_deque;
}

void UWBProcess::pubProcessResult() 
{
    geometry_msgs::PointStamped msg;
    auto result = getProcessResult();

    msg.point.x = result.x;
    msg.point.y = result.y;
    msg.point.z = 0;
    msg.header.frame_id = "veh";
    msg.header.stamp = ros::Time::now();
    
    uwbProcessPub.publish(msg);
}

Point UWBProcess::getProcessResult() 
{
    Point result = {0, 0};
    for(int i = 0; i<PointsDequeFilter.size(); ++i) 
    {
        result.x += PointsDequeFilter[i].x;
        result.y += PointsDequeFilter[i].y;
    }
    result.x /= PointsDequeFilter.size();
    result.y /= PointsDequeFilter.size();
    return result;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_process");
    ros::NodeHandle nh;
    const float Rate = 20;
    
    UWBProcess UWBProcess(nh);

    ros::Rate loop_rate(Rate);
    while(ros::ok())
    {   
        ros::spinOnce();
        
        if(UWBProcess.uwbisok()) 
        {
            UWBProcess.filter();
            UWBProcess.pubProcessResult();
        }

        UWBProcess.clearCallbackFlag();
        loop_rate.sleep();
    }

    return 0;
}