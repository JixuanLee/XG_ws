#include "serial_unpack/inclinometer_serial_manager.h"

namespace Inclinometer
{
/* ======================  InclinometerSerialManager  ====================== */
InclinometerSerialManager::InclinometerSerialManager(ros::NodeHandle & nh, bool & success) : nh_(nh)
{
    inclinometer_pub_ = nh_.advertise<sensor_msgs::Imu>("/inclinometer/raw_data", 10);

    std::string port  = nh_.param<std::string>("inclinometer/port", "/dev/inclinometer");
    uint32_t baudrate = nh_.param("inclinometer/baudrate", 9600);
    uint32_t timeout  = nh_.param("inclinometer/timeout", 2000);

    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(timeout);
    serial_.setPort(port);
    serial_.setBaudrate(baudrate);
    serial_.setTimeout(timeout_);

    try
    {
        serial_.open();
    }
    catch(serial::IOException & e)
    {
        ROS_ERROR("Unable to open port [%s] with baudrate [%u]. Please check port name, baudrate!", port.c_str(), baudrate);
        success = false;
        return;
    }
    if(serial_.isOpen())
    {
        ROS_INFO("Port [%s] with baudrate [%u] is opened.", port.c_str(), baudrate);
    }
    else
    {
        ROS_ERROR("Unable to open port [%s] with baudrate [%u]. Please check port name, baudrate!", port.c_str(), baudrate);
        success = false;
        return;
    }

    success = true;
}

InclinometerSerialManager::~InclinometerSerialManager()
{
    serial_.close();
}

void InclinometerSerialManager::run()
{
    receive_thread_ = std::thread(&InclinometerSerialManager::receiveThread, this);
    receive_thread_.detach();
}

void InclinometerSerialManager::receiveThread()
{
    ros::Rate loop_rate(100);

    std::vector<uint8_t> new_buffer;
    std::vector<uint8_t> last_remaining_buffer;    // 存放上次保留的内容
    std::vector<uint8_t> cat_buffer;               // 存放拼接后的内容
    Inclinometer::Frame data_frame;

    // 一帧长度
    // 1 Byte 设备地址码
    // 1 Byte 功能码
    // 1 Byte 数据长度
    // 52 Byte 数据
    // 2 Byte crc校验
    const int protocol_len = 5 + sizeof(Inclinometer::Frame);    

    while (ros::ok())
    {
        if(serial_.available())
        {
            // 读取数据
            size_t n = serial_.available();
            new_buffer.clear();
            serial_.read(new_buffer, n);

            // 拼接数据
            cat_buffer.clear();
            cat_buffer.insert(cat_buffer.end(), last_remaining_buffer.begin(), last_remaining_buffer.end()); //将last_remaining_buffer压入
            cat_buffer.insert(cat_buffer.end(), new_buffer.begin(), new_buffer.end()); //继续将new_buffer压入
            // last_remaining_buffer.clear();
            size_t data_len = cat_buffer.size();
            
            for (size_t i = 0; i < data_len; i++)
            {
                // modbus 
                // 第一位：设备地址码。只有一个，默认为0x01
                if(cat_buffer[i] == 0x01)
                {
                    // 判断是否存在完整帧
                    if(data_len - i >= protocol_len)
                    {
                        // 判断特定位置数据是否合规
                        // 第二位：功能码。只读取数据，为0x03
                        // 第三位：数据长度。当前定死为52个字节
                        if(cat_buffer[i + 1] != 0x03 || cat_buffer[i + 2] != sizeof(Inclinometer::Frame))
                        {
                            continue;
                        }
                        // 最后两个字节crc校验
                        boost::crc_optimal<16, 0x8005, 0xffff, 0x0000, true, true> crc_modbus_;
                        for (int j = 0; j < protocol_len - 2; j++)
                        {
                            crc_modbus_.process_byte(cat_buffer[i + j]);
                        }
                        uint16_t crc_checksum = cat_buffer[i + protocol_len - 2] | cat_buffer[i + protocol_len - 1] << 8;
                        if (crc_modbus_.checksum() != crc_checksum)
                        {
                            continue;   // crc校验失败
                        }
                        
                        // 得到合规的frame后进行处理
                        data_frame.set_frame(cat_buffer, i + 3);
                        sensor_msgs::Imu msg;
                        msg.header.frame_id = "inclinometer";
                        msg.header.stamp = ros::Time::now();
                        msg.orientation.x = data_frame.get_x_quaternion();
                        msg.orientation.y = data_frame.get_y_quaternion();
                        msg.orientation.z = data_frame.get_z_quaternion();
                        msg.orientation.w = data_frame.get_w_quaternion();
                        msg.angular_velocity.x = data_frame.get_x_angular();
                        msg.angular_velocity.y = data_frame.get_y_angular();
                        msg.angular_velocity.z = data_frame.get_z_angular();
                        msg.linear_acceleration.x = data_frame.get_x_acc();
                        msg.linear_acceleration.y = data_frame.get_y_acc();
                        msg.linear_acceleration.z = data_frame.get_z_acc();
                        // 不要问我为什么这么变换四元数，这个传感器的坐标系很诡异
                        // 我只知道这么变换完，直接用rpy就是车辆坐标系下的rpy
                        // 前提是roll 和 pitch 都在-90deg ~ 90deg内变化（yaw 360deg都可以）。这样才能保证rpy是正确的
                        // 一般情况下roll 和 pitch 都不会超过+-90deg 除非翻车了
                        tf2::Quaternion q;
                        q.setX(msg.orientation.x);
                        q.setY(msg.orientation.y);
                        q.setZ(msg.orientation.z);
                        q.setW(msg.orientation.w);
                        double yaw, pitch, roll;
                        tf2::getEulerYPR(q, pitch, roll, yaw);
                        roll = -roll;
                        pitch = -pitch;
                        yaw -= M_PI_2;
                        if (yaw < -M_PI)
                        {
                            yaw += 2 * M_PI;
                        }
                        msg.orientation_covariance[0] = roll;
                        msg.orientation_covariance[1] = pitch;
                        msg.orientation_covariance[2] = yaw;
                        
                        inclinometer_pub_.publish(msg);
                        // ROS_INFO_THROTTLE(0.2, "roll = %.2f, pitch = %.2f, yaw = %.2f", roll, pitch, yaw);

                        i += protocol_len;
                    }
                    // 如果不存在完整帧，则将剩余数据保存
                    else
                    {
                        last_remaining_buffer.clear();
                        last_remaining_buffer.insert(last_remaining_buffer.end(), cat_buffer.begin() + i, cat_buffer.end()); // 合并两个vector，两帧拼接
                        break;  // 当前内容检测完毕，无完整帧，则直接快进到下一次读取新数据
                    }
                }
            }
        }

        loop_rate.sleep();
    }
}

} // namespace Inclinometer

