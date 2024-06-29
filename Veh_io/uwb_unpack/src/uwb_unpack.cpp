#include <ros/ros.h>
#include <serial/serial.h>
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>

int uwbDataGet(std::vector<uint8_t> & new_buffer, geometry_msgs::PointStamped & uwb_msg, std_msgs::Float32MultiArray & uwb_raw_msg)
{
    static std::vector<uint8_t> last_remaining_buffer; //存放上次保留的内容
    std::vector<uint8_t> cat_buffer; //存放拼接后的内容

    //拼接数据
    cat_buffer.insert(cat_buffer.end(),last_remaining_buffer.begin(),last_remaining_buffer.end()); //将last_remaining_buffer压入
    cat_buffer.insert(cat_buffer.end(),new_buffer.begin(),new_buffer.end()); //继续将new_buffer压入
    int data_len = cat_buffer.size();

    const int protocol_len = 896;   //一帧长度
    const int block_size = 30;      //NLink_LinkTrack_Anchor_Frame0 数据帧中含有的block数目
    const int block_len = 27;       //NLink_LinkTrack_Anchor_Frame0 数据帧中一个block含有的字节数目
    const int anchor_num = 8;

    for(int i = 0; i < data_len; i++)
    {
        // 读取数据帧，帧头固定为 0x55
        // 考虑 NLink_LinkTrack_Anchor_Frame0 这种数据帧， FunctionMask = 0x00
        if(cat_buffer[i] == 0x55 && cat_buffer[i + 1] == 0x00)
        {
            if(data_len - i >= protocol_len)// 判断是否存在完整帧
            {  
                // 验证恒定校验场
                if (cat_buffer[i + 895] != 0xee)
                {
                    // 校验不通过，直接略过当前字节
                    continue;
                }
                
                // 总共30个block，每个block代表一个tag的位置信息
                for (int j = 0; j < block_size; j++)
                {
                    // 每个block第一字节是id， 当id = 0xff 时，代表对应的tag数据无效，直接检索下一个
                    // 当前需求只有一个tag, 因此只有block0存在意义
                    if (cat_buffer[i + j * block_len + 2] == 0xff)
                    {
                        continue;
                    }
                    else
                    {
                        int k = i + j * block_len + 2; // 当前block的头部

                        // 当前需求下，id = 0，即为第0号TAG，通常情况下， k == id       cat_buffer[k] == 0x00
                        // 当前需求下，role = 2， 即为Tag，                          cat_buffer[k + 1] == 0x02
                        // 即目标为Tag0
                        
                        // 以下代码部分，如果有多个tag，需要整体重构，换成数组接收数据
                        // 读取 {pos x, pos y, pox z} * 1000， 但 z 方向误差较大，且目前需求不需要
                        uwb_msg.point.x = ((int32_t)cat_buffer[k + 2] << 8| (int32_t)cat_buffer[k + 3] << 16 | (int32_t)cat_buffer[k + 4] << 24) / 256 / 1000.0;
                        uwb_msg.point.y = ((int32_t)cat_buffer[k + 5] << 8| (int32_t)cat_buffer[k + 6] << 16 | (int32_t)cat_buffer[k + 7] << 24) / 256 / 1000.0;
                        uwb_msg.point.z = ((int32_t)cat_buffer[k + 8] << 8| (int32_t)cat_buffer[k + 9] << 16 | (int32_t)cat_buffer[k +10] << 24) / 256 / 1000.0;
                        // 在当前4个anchor的布置模式下，应当对 x 和 z 坐标的值取反，但 z 是否取不影响本项目。更多细节查看readme
                        uwb_msg.point.x = -uwb_msg.point.x;
                        uwb_msg.point.z = -uwb_msg.point.z;

                        // 读取 {dis0, dis1, dis2, dis3, dis4, dis5, dis6, dis7} * 100
                        // 当前需求下主要是利用到 a0 a1 a2 a3 四个基站的距离，也就是data[0] data[1] data[2] data[3]
                        uwb_raw_msg.data.resize(anchor_num);
                        for (int ii = 0; ii < anchor_num; ii++)
                        {
                            uwb_raw_msg.data[ii] = ((uint16_t)cat_buffer[k + 11 + 2*ii] | (uint16_t)cat_buffer[k + 12 + 2*ii] << 8) / 100.0;
                        }   
                    }
                }
                
                // 保留位       i + 2 + block_size * block_len ~ i + 2 + block_size * block_len + 67 (812~878)之间为保留位
                // local_time 设备本地时间，单位ms      （879～882）
                // 保留位                              (883~886)
                // 电压*1000，单位V                     (887~888)
                // system_time 系统时间，单位ms          (889~892)
                // 本设备id 由于用Anchor0接收数据，应该是0 (893)
                // 本设备role 由于是Anchor，应该是0       (894)
 
                // 将剩余数据保留，
                last_remaining_buffer.clear();
                last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer.begin()+i+protocol_len, cat_buffer.end()); // 合并两个vector，两帧拼接
                i += protocol_len;
            }
            else
            {
                // 如果不存在完整帧，则将剩余数据保存
                last_remaining_buffer.clear();
                last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer.begin(), cat_buffer.end()); // 合并两个vector，两帧拼接
                return 0;
            }
        }
    }

    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_unpack");
    ros::NodeHandle nh;
    const float Rate = 20;

    ros::Publisher uwbPub = nh.advertise<geometry_msgs::PointStamped>("uwb",10);
    geometry_msgs::PointStamped uwb_msg;
    geometry_msgs::Point uwb_msg_old;
    uwb_msg.header.frame_id = "veh";
    ros::Publisher uwbRawPub = nh.advertise<std_msgs::Float32MultiArray>("uwb_raw",10);
    std_msgs::Float32MultiArray uwb_raw_msg;

    serial::Serial serial;
    serial::Timeout timeOut = serial::Timeout::simpleTimeout(100);
    serial.setPort("/dev/usb_anchor0");
    serial.setBaudrate(921600);
    serial.setTimeout(timeOut);
    size_t n;
    std::vector<uint8_t> new_buffer;
    
    //打开串口
    try
    {
        serial.open();
    }
    catch(serial::IOException & e)
    {
        ROS_ERROR("Unable to open port. Can't read Tag information.");
        return -1;
    }
    //判断串口是否打开成功
    if(serial.isOpen())
    {
        ROS_INFO("/dev/usb_anchor0 is opened.");
    }
    else
    {
        return -1;
    }
    
    std::chrono::steady_clock::time_point currentTime, lastUpdateTime;
    std::chrono::milliseconds elapsedTime;
    std::chrono::milliseconds filterDelay_Tag(100); // 过滤器延迟时间，从usb中一段时间接收不到数据，就掐断。
    lastUpdateTime = std::chrono::steady_clock::now();
    ros::Rate loop_rate(Rate);
    while(ros::ok())
    {
        currentTime = std::chrono::steady_clock::now();
        elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastUpdateTime);
        if(serial.available())
        {
            n = serial.available();
            new_buffer.clear();
            serial.read(new_buffer, n); 
            uwbDataGet(new_buffer, uwb_msg, uwb_raw_msg);
            
            if ( ! (uwb_msg.point.x == uwb_msg_old.x && uwb_msg.point.y == uwb_msg_old.y && uwb_msg.point.z == uwb_msg_old.z))     
            {
                uwb_msg.header.stamp = ros::Time::now();
                uwbPub.publish(uwb_msg);
                uwbRawPub.publish(uwb_raw_msg);

                //基站数为1或2时的特殊情况，目标点被设定为111
                //读不到数时，（打开的串口有误），导致数据一直为000
                //过程中有两个非anchor0的基站断开时，硬件数据会一直锁死为上一次有效数据一直发送。
                uwb_msg_old.x = uwb_msg.point.x;
                uwb_msg_old.y = uwb_msg.point.y;
                uwb_msg_old.z = uwb_msg.point.z;
                lastUpdateTime = std::chrono::steady_clock::now();
            }
            else
            {
                if (elapsedTime >= filterDelay_Tag)
                {
                    ROS_ERROR("lose connection with UWB anchors, shut down!");
                    return -1;
                }
            }
        }
        else
        {
            if (elapsedTime >= filterDelay_Tag)
            {
                ROS_ERROR("lose connection with UWB anchor0 or tag, shut down!");
                return -1;
            }
        }
        
        loop_rate.sleep();
    }
    //关闭串口
    serial.close();
    return 0;
}
