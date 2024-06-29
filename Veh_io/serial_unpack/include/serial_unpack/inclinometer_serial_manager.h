#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <thread>
#include <boost/crc.hpp>

namespace Inclinometer
{
class Frame
{
public:
    Frame(){};
    ~Frame(){};

    void set_frame(const std::vector<uint8_t> & buffer, int pos)
    {
        memcpy(&data, &buffer[pos], sizeof(_Frame));
    }

    /// @brief x 方向角度 单位：deg
    // float get_x_angle()
    // {
    //     return (data.x_angle - 20000) / 100.0f;
    // }

    /// @brief y 方向角度 单位：deg
    // float get_y_angle()
    // {
    //     return (data.y_angle - 20000) / 100.0f;
    // }

    /// @brief z 方向角度 单位：deg
    // float get_z_angle()
    // {
    //     return (data.z_angle - 20000) / 100.0f;
    // }

    /// @brief x 方向加速度 单位：m/s^2
    float get_x_acc()
    {
        uint32_t temp = static_cast<uint32_t>(data.x_acc_1 | data.x_acc_2 << 8 | data.x_acc_3 << 16 | data.x_acc_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return -ret * 9.81;
    }

    /// @brief y 方向加速度 单位：m/s^2
    float get_y_acc()
    {
        uint32_t temp = static_cast<uint32_t>(data.y_acc_1 | data.y_acc_2 << 8 | data.y_acc_3 << 16 | data.y_acc_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return -ret * 9.81;
    }

    /// @brief z 方向加速度 单位：m/s^2
    float get_z_acc()
    {
        uint32_t temp = static_cast<uint32_t>(data.z_acc_1 | data.z_acc_2 << 8 | data.z_acc_3 << 16 | data.z_acc_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        // 只受重力时，输出为-9.8
        return -ret * 9.81;
    }

    /// @brief x 方向角速度 单位：rad/s
    float get_x_angular()
    {
        // 坐标系变换
        uint32_t temp = static_cast<uint32_t>(data.y_angular_1 | data.y_angular_2 << 8 | data.y_angular_3 << 16 | data.y_angular_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret / 180 * M_PI;
    }

    /// @brief y 方向角速度 单位：rad/s
    float get_y_angular()
    {
        // 坐标系变换
        uint32_t temp = static_cast<uint32_t>(data.x_angular_1 | data.x_angular_2 << 8 | data.x_angular_3 << 16 | data.x_angular_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return -ret / 180 * M_PI;
    }

    /// @brief z 方向角速度 单位：rad/s
    float get_z_angular()
    {
        uint32_t temp = static_cast<uint32_t>(data.z_angular_1 | data.z_angular_2 << 8 | data.z_angular_3 << 16 | data.z_angular_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret / 180 * M_PI;
    }

    /// @brief x 方向磁场
    float get_x_magnetic()
    {
        // 坐标系变换
        uint32_t temp = static_cast<uint32_t>(data.y_magnetic_1 | data.y_magnetic_2 << 8 | data.y_magnetic_3 << 16 | data.y_magnetic_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret;
    }

    /// @brief y 方向磁场
    float get_y_magnetic()
    {
        // 坐标系变换
        uint32_t temp = static_cast<uint32_t>(data.x_magnetic_1 | data.x_magnetic_2 << 8 | data.x_magnetic_3 << 16 | data.x_magnetic_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return -ret;
    }

    /// @brief z 方向磁场
    float get_z_magnetic()
    {
        uint32_t temp = static_cast<uint32_t>(data.z_magnetic_1 | data.z_magnetic_2 << 8 | data.z_magnetic_3 << 16 | data.z_magnetic_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret;
    }

    /// @brief x 方向四元数
    float get_x_quaternion()
    {
        uint32_t temp = static_cast<uint32_t>(data.x_quaternion_1 | data.x_quaternion_2 << 8 | data.x_quaternion_3 << 16 | data.x_quaternion_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret;
    }

    /// @brief y 方向四元数
    float get_y_quaternion()
    {
        uint32_t temp = static_cast<uint32_t>(data.y_quaternion_1 | data.y_quaternion_2 << 8 | data.y_quaternion_3 << 16 | data.y_quaternion_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret;
    }

    /// @brief z 方向四元数
    float get_z_quaternion()
    {
        uint32_t temp = static_cast<uint32_t>(data.z_quaternion_1 | data.z_quaternion_2 << 8 | data.z_quaternion_3 << 16 | data.z_quaternion_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret;
    }

    /// @brief w 方向四元数
    float get_w_quaternion()
    {
        uint32_t temp = static_cast<uint32_t>(data.w_quaternion_1 | data.w_quaternion_2 << 8 | data.w_quaternion_3 << 16 | data.w_quaternion_4 << 24);
        float ret;
        memcpy(&ret, &temp, 4);
        return ret;
    }

private:
    // 52 Bytes
    struct _Frame
    {
        uint8_t x_acc_1;
        uint8_t x_acc_2;
        uint8_t x_acc_3;
        uint8_t x_acc_4;
        uint8_t y_acc_1;
        uint8_t y_acc_2;
        uint8_t y_acc_3;
        uint8_t y_acc_4;
        uint8_t z_acc_1;
        uint8_t z_acc_2;
        uint8_t z_acc_3;
        uint8_t z_acc_4;

        uint8_t x_angular_1;
        uint8_t x_angular_2;
        uint8_t x_angular_3;
        uint8_t x_angular_4;
        uint8_t y_angular_1;
        uint8_t y_angular_2;
        uint8_t y_angular_3;
        uint8_t y_angular_4;
        uint8_t z_angular_1;
        uint8_t z_angular_2;
        uint8_t z_angular_3;
        uint8_t z_angular_4;

        uint8_t x_magnetic_1;
        uint8_t x_magnetic_2;
        uint8_t x_magnetic_3;
        uint8_t x_magnetic_4;
        uint8_t y_magnetic_1;
        uint8_t y_magnetic_2;
        uint8_t y_magnetic_3;
        uint8_t y_magnetic_4;
        uint8_t z_magnetic_1;
        uint8_t z_magnetic_2;
        uint8_t z_magnetic_3;
        uint8_t z_magnetic_4;

        uint8_t x_quaternion_1;
        uint8_t x_quaternion_2;
        uint8_t x_quaternion_3;
        uint8_t x_quaternion_4;
        uint8_t y_quaternion_1;
        uint8_t y_quaternion_2;
        uint8_t y_quaternion_3;
        uint8_t y_quaternion_4;
        uint8_t z_quaternion_1;
        uint8_t z_quaternion_2;
        uint8_t z_quaternion_3;
        uint8_t z_quaternion_4;
        uint8_t w_quaternion_1;
        uint8_t w_quaternion_2;
        uint8_t w_quaternion_3;
        uint8_t w_quaternion_4;
    };

    _Frame data;
};
    
class InclinometerSerialManager
{
/*
    北微传感 倾角传感器 型号AH127C-485
    坐标系  对于线加速度 正前方为x 左侧为y 上方为z 输出的原始数据表示为 重力加速度g 的倍数
           对于角速度 正前方为y正方向 右侧为x正方向 上方为z正方向 满足右手坐标系
           对于角度 xy方向与上述一致，数据范围(-180deg ~ 180deg) z方向 正朝北时theta_z为0 顺时针增加，数据范围(0deg ~ 360deg)
    为保持坐标系一致性，在解包过程中，就将倾角传感器的坐标系变换成与车辆坐标系一致，即x正前方 y左侧 z上方的右手坐标系

    01 06 00 1B 00 0a 79 ca
*/
public:
    InclinometerSerialManager(ros::NodeHandle & nh, bool & success);
    ~InclinometerSerialManager();

    void run();

private:
    ros::NodeHandle nh_;
    serial::Serial serial_;
    ros::Publisher inclinometer_pub_;

    std::thread receive_thread_;
    void receiveThread();
};

} // namespace Inclinometer
