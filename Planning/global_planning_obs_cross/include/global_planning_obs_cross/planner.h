#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.
   为hybrid A*算法创建接口的类，从`ros::nav_core::BaseGlobalPlanner`派生，很容易集成到ROS Navigation Stack
    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();//默认构造器

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();//初始化碰撞检测器和启发式函数用到的搜索查找表

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);//通过订阅者监听的回调函数设置地图

  /*!
     \brief setGoal
     \param goal the goal pose 设置目标pose，即设置终点
  */
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  void TimerCallback(const ros::TimerEvent &);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
     核心规划函数
  */
  void plan();

 private:
  /// The node handle
  ros::NodeHandle n; //节点句柄
  ros::Subscriber subMap;//接收地图信息的订阅器
  ros::Subscriber subGoal;//接收目标更新的订阅器
  std::string input_map_topic; // ljx
  std::string input_goal_topic; // ljx

  bool has_obs = false;
  bool finish_obs_cross = false;
  float obs_x = 0;
  float obs_y = 0;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
  /// The path produced by the hybrid A* algorithm
  Path path;//生成混合A*路径的实体对象
  /// The smoother used for optimizing the path
  Smoother smoother;//路径平滑实体
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);//用于发布给控制器的平滑路径
  /// The visualization used for search visualization
  Visualize visualization;//可视化对象，与RVIZ交互
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram; //Voroni Diagram
  
  //用来存储RVIZ的结果
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::Pose start;
  /// The goal pose set through RViz
  geometry_msgs::Pose goal;
  /// 
  bool got_map = false;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;

  //查找表：用于碰撞的查找表及Dubin PATH的查找表
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions]; 
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
