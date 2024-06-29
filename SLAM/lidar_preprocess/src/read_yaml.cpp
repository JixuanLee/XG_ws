#include "lidar_preprocess/read_yaml.h"


ReadYaml::ReadYaml(std::string dir)
{
    config_ = YAML::LoadFile(dir);

    try{
        min_dist_ = config_["ground_remove_and_point_concat"]["min_dist"].as<float>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("min_dist param get error");
    }
    try{
        leaf_size_ = config_["ground_remove_and_point_concat"]["leaf_size"].as<float>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("leaf_size_ param get error");
    }
    try{
        cell_size_ = config_["ground_remove_and_point_concat"]["cell_size"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("cell_size_ param get error");
    }
    try{
        z_threshold_ = config_["ground_remove_and_point_concat"]["z_threshold"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("z_threshold_ param get error");
    }
    try{
        y_threshold_ = config_["ground_remove_and_point_concat"]["y_threshold"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("y_threshold_ param get error");
    }
    try{
        x_threshold_ = config_["ground_remove_and_point_concat"]["x_threshold"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("x_threshold_ param get error");
    }
    try{
        ground_remove_threshold_ = config_["ground_remove_and_point_concat"]["ground_remove_threshold"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("ground_remove_threshold_ param get error");
    }
    try{
        ground_z_ = config_["ground_remove_and_point_concat"]["ground_z"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("ground_z_ param get error");
    }
    try{
        ground_beyond_threshold_ = config_["ground_remove_and_point_concat"]["ground_beyond_threshold"].as<double>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("ground_beyond_threshold_ param get error");
    }
    try{
        middle_lidar_sub_topic_ = config_["ground_remove_and_point_concat"]["middle_lidar_sub_topic"].as<std::string>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("middle_lidar_sub_topic_ param get error");
    }
    try{
        front_lidar_sub_topic_ = config_["ground_remove_and_point_concat"]["front_lidar_sub_topic"].as<std::string>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("middle_lidar_sub_topic_ param get error");
    }
    try{
        left_lidar_sub_topic_ = config_["ground_remove_and_point_concat"]["left_lidar_sub_topic"].as<std::string>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("left_lidar_sub_topic_ param get error");
    }
    try{
        right_lidar_sub_topic_ = config_["ground_remove_and_point_concat"]["right_lidar_sub_topic"].as<std::string>();
    }catch(const YAML::Exception &e){
        ROS_ERROR("right_lidar_sub_topic_ param get error");
    }

    // try{
    //     std::vector<double> T(16, 0.0);
    //     for (int i = 0; i < 16; i++){
    //             T[i] = config_["ground_remove_and_point_concat"]["T_middle2veh"][i].as<double>();
    //     }
    //     T_middle2veh_ << T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9],
    //     T[10], T[11], T[12], T[13], T[14], T[15];
    // }catch(const YAML::Exception &e){
    //     ROS_ERROR("T_middle2veh_ param get error");
    // }

    try{
        std::vector<double> T(16, 0.0);
        for (int i = 0; i < 16; i++){
                T[i] = config_["ground_remove_and_point_concat"]["T_front2middle"][i].as<double>();
        }
        T_front2middle_ << T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9],
        T[10], T[11], T[12], T[13], T[14], T[15];
    }catch(const YAML::Exception &e){
        ROS_ERROR("T_front2middle_ param get error");
    }

    try{
        std::vector<double> T(16, 0.0);
        for (int i = 0; i < 16; i++){
                T[i] = config_["ground_remove_and_point_concat"]["T_left2middle"][i].as<double>();
        }
        T_left2middle_ << T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9],
        T[10], T[11], T[12], T[13], T[14], T[15];
    }catch(const YAML::Exception &e){
        ROS_ERROR("T_left_ param get error");
    }

    try{
        std::vector<double> T(16, 0.0);
        for (int i = 0; i < 16; i++){
                T[i] = config_["ground_remove_and_point_concat"]["T_right2middle"][i].as<double>();
        } 
        T_right2middle_ << T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8], T[9],
        T[10], T[11], T[12], T[13], T[14], T[15];
    }catch(const YAML::Exception &e){
        ROS_ERROR("T_right_ param get error");
    }

}

ReadYaml::~ReadYaml()
{

}