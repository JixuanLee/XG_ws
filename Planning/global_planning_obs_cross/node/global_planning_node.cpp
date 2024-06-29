#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "global_planning_obs_cross/constants.h"
#include "global_planning_obs_cross/planner.h"
using namespace HybridAStar;

// 初始化定义extern变量
bool Constants::reverse = 0;
bool Constants::dubinsShot = 0;
bool Constants::dubins = 0;
bool Constants::dubinsLookup = 0;
bool Constants::twoD = 0;
int Constants::iterations = 0;
double Constants::bloating = 0;
double Constants::width = 0;
double Constants::length = 0;
float Constants::r = 0;
float Constants::cellSize = 0;
float Constants::deltaHeadingDeg = 0;
float Constants::deltaHeadingRad = 0;
float Constants::deltaHeadingNegRad = 0;
float Constants::tieBreaker = 0;
float Constants::factor2D = 0;
float Constants::penaltyTurning = 0;
float Constants::penaltyReversing = 0;
float Constants::penaltyCOD = 0;
float Constants::dubinsShotDistance = 0;
float Constants::dubinsStepSize = 0;
int Constants::dubinsWidth = 0;
int Constants::dubinsArea = 0;
int Constants::bbSize = 0;
float Constants::minRoadWidth = 0;


template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "global_planning_obs_cross");
  ros::NodeHandle n("~");

  // -------------------------↓↓↓↓↓↓↓↓从launch初始化参数  -- ljx0414  ↓↓↓↓↓↓↓↓-------------------------
  // <!-- 1.标志位 -->
  n.param("reverse", Constants::reverse, false);
  n.param("dubinsShot", Constants::dubinsShot, true);
  n.param("dubins", Constants::dubins, true);
  n.param("dubinsLookup", Constants::dubinsLookup, false);
  n.param("twoD", Constants::twoD, false);
  Constants::dubinsLookup = Constants::dubinsLookup && Constants::dubins;

  // <!-- 2.通用参数 -->
  n.param("iterations", Constants::iterations, 10000000); 
  n.param("bloating", Constants::bloating, 0.5);
  n.param("width", Constants::width, 2.1);
  n.param("length", Constants::length, 3.2);
  n.param("r", Constants::r, float(5.4)); 
  Constants::width = Constants::width + 2* Constants::bloating;
  Constants::length = Constants::length + 2* Constants::bloating;
  Constants::deltaHeadingDeg = 360 / (float)Constants::headings; 
  Constants::deltaHeadingRad = 2 * M_PI /(float)Constants::headings; 
  Constants::deltaHeadingNegRad = 2* M_PI - Constants::deltaHeadingRad; 
  n.param("cellSize", Constants::cellSize, float(0.4));
  n.param("tieBreaker", Constants::tieBreaker, float(0.1));

  // <!-- 3.启发式常数 -->
  n.param("factor2D", Constants::factor2D, float(3.449489)); 
  n.param("penaltyTurning", Constants::penaltyTurning, float(1.2));
  n.param("penaltyReversing", Constants::penaltyReversing, float(2.0));
  n.param("penaltyCOD", Constants::penaltyCOD, float(2.0));
  n.param("dubinsShotDistance", Constants::dubinsShotDistance, float(400));
  n.param("dubinsStepSize", Constants::dubinsStepSize, float(5));

  // <!-- 4.其他设置 -->
  n.param("dubinsWidth", Constants::dubinsWidth, 100); 
  n.param("minRoadWidth", Constants::minRoadWidth, float(2.2));
  Constants::dubinsArea = Constants::dubinsWidth * Constants::dubinsWidth;
  Constants::bbSize = std::ceil((sqrt(Constants::width * Constants::width + Constants::length* Constants::length)) / Constants::cellSize);

  // 其他没有写成动态传参的，在constants.h中修改，如heading等。
  // -------------------------↑↑↑↑↑↑↑↑从launch初始化参数  -- ljx0414  ↑↑↑↑↑↑↑↑-------------------------


  HybridAStar::Planner hy;

  ros::spin();
  return 0;
}
