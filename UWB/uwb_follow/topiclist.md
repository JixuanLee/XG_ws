# 话题列表：     

## 1.订阅话题
话题                 | 数据类型                       | 作用
----                | -----                         | ------  
/uwb                | geometry_msgs::PointStamped   | 获得uwb硬件算出的tag xy数据，用于后续定位跟随  
/uwb_process        | geometry_msgs::PointStamped   | 获得处理过的uwb数据，用于后续定位跟随  
/odom               | nav_msgs::Odometry            | 获得里程计信息,暂时不用  
/chassis            | std_msgs::Bool                | 判断是否能与底盘保持联通,用于后续心跳判断
/lidar_raw          | sensor_msgs::PointCloud2      | 获得激光雷达的点云数据，用于后续建图  

## 2.发布话题
话题                 | 数据类型                       | 作用
----                | -----                         | ------  
/cmd_raw            | std_msgs::Float32MultiArray   | 根据目标点位置，利用pid计算得到用于控制车辆的线速度和角速度
/uwb_target         | geometry_msgs::PoseStampe     | 利用/uwb_process的目标点和车辆当前状态中和目标的切向角，发布新的目标位置
/brake              | std_msgs::Bool                | 根据uwb跟随效果发布的用于判断是否进行停车的标志
/rear               | std_msgs::Bool                | 根据uwb跟随效果发布的用于判断是否进行倒车的标志
/local_map          | nav_msgs::OccupancyGrid       | 利用激光雷达进行局部建图，二值化地图
/laser_points       | geometry_msgs::PoseArray      | 获取激光雷达点后进行滤波处理，得到的周围环境点云
/laser_edge_points  | geometry_msgs::PoseArray      | 获取激光雷达点后进行滤波处理，找到周围边缘点并发布