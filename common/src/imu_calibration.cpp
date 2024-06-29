/*********************************************************
功能：消除imu安装误差，完成imu标定，获得一个变换矩阵与比例矩阵

**********************************************************/

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>


std::string imu_topic_name;
int point_num;
double gravity;
std::vector<double> accX, accY, accZ;


double GetSum(std::vector<double> &msg)
{
    double sum = 0;
    for (size_t i = 0; i < msg.size(); i++)
    {
        sum += msg[i];
    }
    return sum;
}

// 计算旋转轴
// 就是旋转前后两个向量的叉积向量的方向
inline Eigen::Vector3d calculateRotAxis(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    return Eigen::Vector3d(vectorBefore.y() * vectorAfter.z() - vectorBefore.z() * vectorAfter.y(),
                           vectorBefore.z() * vectorAfter.x() - vectorBefore.x() * vectorAfter.z(),
                           vectorBefore.x() * vectorAfter.y() - vectorBefore.y() * vectorAfter.x());
}

// 计算旋转角 rad
// 两个向量的内积公式变形即可得到
double calculateAngle(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    double ab, a1, b1, cosr;
    ab = vectorBefore.x() * vectorAfter.x() + vectorBefore.y() * vectorAfter.y() + vectorBefore.z() * vectorAfter.z();
    a1 = sqrt(vectorBefore.x() * vectorBefore.x() + vectorBefore.y() * vectorBefore.y() + vectorBefore.z() * vectorBefore.z());
    b1 = sqrt(vectorAfter.x() * vectorAfter.x() + vectorAfter.y() * vectorAfter.y() + vectorAfter.z() * vectorAfter.z());
    cosr = ab / a1 / b1;
    return acos(cosr);
}

// 计算旋转矩阵
void rotationMatrix(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter, Eigen::Matrix3d &rotMatrix)
{
    Eigen::Vector3d vector = calculateRotAxis(vectorBefore, vectorAfter);
    double angle = calculateAngle(vectorBefore, vectorAfter);
    Eigen::AngleAxisd rotationVector(angle, vector.normalized());
    rotMatrix = rotationVector.toRotationMatrix();
}

void imuCallback(const sensor_msgs::Imu &imuIn)
{
    ROS_INFO_THROTTLE(60, "imu calibration: %lu / %d", accX.size(), point_num);

    if (accX.size() < point_num || accY.size() < point_num || accZ.size() < point_num)
    {
        accX.push_back(imuIn.linear_acceleration.x);
        accY.push_back(imuIn.linear_acceleration.y);
        accZ.push_back(imuIn.linear_acceleration.z);
    }
    else
    {
        double accXMean = GetSum(accX) / accX.size();
        double accYMean = GetSum(accY) / accY.size();
        double accZMean = GetSum(accZ) / accZ.size();

        double accScale = gravity / sqrt(pow(accXMean, 2) + pow(accYMean, 2) + pow(accZMean, 2));

        accXMean *= accScale;
        accYMean *= accScale;
        accZMean *= accScale;

        Eigen::Matrix3d rotMatrix;
        Eigen::Vector3d vectorBefore(accXMean, accYMean, accZMean);
        Eigen::Vector3d vectorAfter(0, 0, gravity);

        rotationMatrix(vectorBefore, vectorAfter, rotMatrix);

        std::string save_path = ros::package::getPath("can_unpack").append("/config/imu.yaml");

        YAML::Node out;
        out["name"] = "imu_calibration_config";
        out["acceleration_scale"] = accScale;
        YAML::Node rotation_matrix_node;
        for (int i = 0; i < rotMatrix.rows(); i++)
        {
            for (int j = 0; j < rotMatrix.cols(); j++)
            {
                rotation_matrix_node.push_back(rotMatrix(i, j));
            }
        } 
        out["rotation_matrix"] = rotation_matrix_node;

        std::ofstream fout(save_path, std::ios::trunc);
        if (fout.is_open())
        {
            fout << out << std::endl;
            fout.close();
            ROS_INFO_STREAM("imu calibration result has saved to " << save_path);
        } 
        else 
            ROS_WARN("fail to save imu calibration result!");    

        // std::cout << rotMatrix << std::endl;
        // std::cout << std::endl;
        // std::cout << rotMatrix*vectorBefore << std::endl;
        // std::cout <<  std::endl;
        // std::cout << vectorBefore << std::endl;
        // std::cout <<  std::endl;
        // std::cout << vectorAfter << std::endl;

        exit(0);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_calibration");
    ros::NodeHandle nh("~");
    imu_topic_name = nh.param("imu_topic_name", std::string("/imu/raw_data"));
    point_num = nh.param("point_num", 12000);
    gravity = nh.param("gravity", 9.81);

    ros::Subscriber subImu = nh.subscribe(imu_topic_name, 20, imuCallback);

    ros::spin();
}