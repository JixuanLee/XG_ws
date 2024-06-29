#ifndef READ_YMAL_HPP
#define READ_YAML_HPP

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

using namespace std;
class ReadYaml
{
private:
    YAML::Node config_;

    float min_dist_;
    float leaf_size_;
    float cell_size_;
    float z_threshold_;
    float y_threshold_;
    float x_threshold_;
    float ground_remove_threshold_;
    float ground_z_;
    float ground_beyond_threshold_;

    std::string middle_lidar_sub_topic_;
    std::string front_lidar_sub_topic_;
    std::string left_lidar_sub_topic_;
    std::string right_lidar_sub_topic_;

    // Eigen::Matrix4d T_middle2veh_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_front2middle_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_left2middle_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_right2middle_ = Eigen::Matrix4d::Identity();

public:
    ReadYaml(std::string dir);
    ~ReadYaml();

    float ReturnMinDist(){
        return min_dist_;
    }

    float ReturnLeftSize(){
        return leaf_size_;
    }

    float ReturnCellSize(){
        return cell_size_;             
    }

    float Returnzthreshold(){
        return z_threshold_;
    }

    float ReturnYThreshold(){
        return y_threshold_;
    }

    float ReturnXThreshold(){
        return x_threshold_;
    }

    float ReturnGroundRemoveThreshold(){
        return ground_remove_threshold_;
    }

    float ReturnGroundZ(){
        return ground_z_;
    }

    float ReturnGroundBeyondThreshold(){
        return ground_beyond_threshold_;
    }

    string ReturnMiddleLidarSubTopic(){
        return middle_lidar_sub_topic_;
    };

    string ReturnFrontLidarSubTopic(){
        return front_lidar_sub_topic_;
    };

    string ReturnLeftLidarSubTopic(){
        return left_lidar_sub_topic_;
    };

    string ReturnRightLidarSubTopic(){
        return right_lidar_sub_topic_;
    };

    // Eigen::Matrix4d ReturnMiddle2Veh(){
    //     return T_middle2veh_;
    // }

    Eigen::Matrix4d ReturnFront2Middle(){
        return T_front2middle_;
    }

    Eigen::Matrix4d ReturnLeft2Middle(){
        return T_left2middle_;
    }

    Eigen::Matrix4d ReturnRight2Middle(){
        return T_right2middle_;
    }

};

#endif  // READ_YAML_HPP