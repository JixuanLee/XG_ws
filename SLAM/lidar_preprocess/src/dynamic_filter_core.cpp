// 这是动态点滤除的程序。
// 更新：李季轩+2023-11-30：更新了坐标系变换关系，调整了发布的点云坐标系，优化了打印。
// 输入：预处理后的拼接点云+IMU/A-LO提供的odom&tf数据。
// 输出：对输入点云处理后得到的：动态点云（雷达坐标系）+静态点云（地图坐标系，好点、可用）+ 栅格地图（目前是实时、局部，未保存本地）。
// 原理：利用odom进行栅格地图实时局部更新，利用tf进行点云坐标对齐，基于栅格内点云数量进行栅格占有率的增减，
// 原理：再根据每个栅格内占有率的多寡判断该栅格内的点云是静态点还是动态点。
// 后端：将静态点云输入SLAM算法进行建图。
// 升级：2024-04-11：引入前端点云预处理的“地面点”，在预留接口的前提下提供了输出 [含地面点] 或 [不含地面点] 的静态点云的选项。

#include "lidar_preprocess/dynamic_filter.h"

// ljx 2024-04-15 为了修复SLAM初始阶段，由于预处理/动态点剔除等节点启动原因，导致的杂点未完全滤除问题，选择在 [为SLAM输入点云的节点] 里drop掉前几帧数据的pub。
#define DELAYTIMES2PUB 10

DynamicFilter::DynamicFilter(void) : local_nh_("~"), nh_(), listener_(tf_buffer_)
{
    nh_.param("/dynamic_filter/resolution", resolution_, {0.2});
    nh_.param("/dynamic_filter/width", width_, {40.0});
    nh_.param("/dynamic_filter/occupancy_threshold", occupancy_threshold_, {0.2});
    nh_.param("/dynamic_filter/beam_num", beam_num_, {720});
    nh_.param("/dynamic_filter/log_odds_increase", log_odds_increase_, {0.4});
    nh_.param("/dynamic_filter/log_odds_decrease", log_odds_decrease_, {0.2});
    nh_.param("/dynamic_filter/input_pointcloud_topic", input_pointcloud_topic_, std::string("/all_lidars_preprocessed_ros"));
    nh_.param("/dynamic_filter/input_pointcloud_ground_topic", input_pointcloud_ground_topic_, std::string("/all_lidars_preprocessed_ground_ros"));
    nh_.param("/dynamic_filter/input_odom_topic", input_odom_topic_, std::string("/odom_by_imu_gnss"));    
    nh_.param("/dynamic_filter/doMurgeTheGroundPoints", doMurgeTheGroundPoints_, bool(true));    

    float lineRes = 0.8;
	downSizeFilterStatic.setLeafSize(lineRes, lineRes,lineRes);

    grid_width_ = width_ / resolution_;
    grid_num_ = grid_width_ * grid_width_;
    width_2_ = width_ / 2.0;
    grid_width_2_ = grid_width_ / 2.0;

    dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_point_ros", 10);
    static_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/static_point_ros", 10);

    grid_pub_ = local_nh_.advertise<nav_msgs::OccupancyGrid>("/local_occupancy_grid", 10);

    // 订阅回调，有2套，对应[doMurgeTheGroundPoints_]参数的功能（详见launch）
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_cloud_sub_
                (nh_, input_pointcloud_topic_, 50, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::PointCloud2> obstacles_cloud_ground_sub_
                (nh_, input_pointcloud_ground_topic_, 50, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_
                (nh_, input_odom_topic_, 50, ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_subs;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_do_murge_subs;
    message_filters::Synchronizer<sync_subs> sync_(sync_subs(100), obstacles_cloud_sub_, odom_sub_);
    message_filters::Synchronizer<sync_do_murge_subs> sync_do_murge_(sync_do_murge_subs(100), obstacles_cloud_sub_, obstacles_cloud_ground_sub_, odom_sub_);
    sync_do_murge_.registerCallback(boost::bind(&DynamicFilter::callback_do_murge_ground, this, _1, _2, _3));
    sync_.registerCallback(boost::bind(&DynamicFilter::callback, this, _1, _2));

    occupancy_grid_map_.resize(grid_num_);

    std::cout << "\033[32m======== Dynamic Filter ========" << std::endl;
    std::cout << "\033[32mresolution: " << resolution_ << std::endl;
    std::cout << "\033[32mwidth: " << width_ << std::endl;
    std::cout << "\033[32mwidth_2:  " << width_2_ << std::endl;
    std::cout << "\033[32mgrid_num: " << grid_num_ << std::endl;
    std::cout << "\033[32moccupancy_threshold: " << occupancy_threshold_ << std::endl;
    std::cout << "\033[32mlog_odds_increase: " << log_odds_increase_ << std::endl;
    std::cout << "\033[32mlog_odds_decrease: " << log_odds_decrease_ << std::endl;
    std::cout << "\033[32minput_pointcloud_topic: " << input_pointcloud_topic_ << std::endl;
    std::cout << "\033[32minput_pointcloud_ground_topic: " << input_pointcloud_ground_topic_ << std::endl;
    std::cout << "\033[32minput_odom_topic: " << input_odom_topic_ << std::endl;
    std::cout << "\033[32mdoMurgeTheGroundPoints: " << doMurgeTheGroundPoints_ << std::endl;

    start_process();
}

DynamicFilter::~DynamicFilter()
{
}

void DynamicFilter::callback(const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud, 
                                                            const nav_msgs::OdometryConstPtr& msg_odom)
{
    if (doMurgeTheGroundPoints_) // 在这里而非在“回调的声明”进行if判断，是因为在声明处判断会导致“声明失效”，无法进入回调 - ljx0411测试
        return;
    std::cout << "********************Epoch Is Begin *******************" << std::endl;

    // ljx理解备注：
    // 使用odom话题数据，是为了让栅格地图trans，应该使用map->veh的实际odom
    // 使用tf广播数据，是为了让点云trans，应该使用veh->lidar的实际tf
    const double start_time = ros::Time::now().toSec();
    static Eigen::Vector2d last_odom_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y);
    static double last_yaw = tf2::getYaw(msg_odom->pose.pose.orientation);

    std::cout << "--------[0]  DynamicFilter Callback --------" << std::endl;
    CloudXYZIPtr cloud_ptr(new CloudXYZI);
    pcl::fromROSMsg(*msg_obstacles_cloud, *cloud_ptr);

    std::cout << "      received cloud without ground size:     " << cloud_ptr->points.size() << std::endl;

    // 原始点云坐标系, 雷达的坐标系(与/rslidar_link等价的坐标系)
    const std::string sensor_frame_id = remove_first_slash(msg_obstacles_cloud->header.frame_id);
    std::cout << "      [L] sensor_frame_id:     " << sensor_frame_id << std::endl;

    // 目标坐标系，车体坐标系(与/veh等价的坐标系)
    const std::string base_frame_id = remove_first_slash(msg_odom->child_frame_id); 
    std::cout << "      [V] base_frame_id:     " << base_frame_id << std::endl; //如果frame不对，检查上一行是child_frame_id, 还是header.frame_id


    // 将雷达实时点云，从sensor_frame（雷达frame）转到base_frame（车体frame）下
    try{
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer_.lookupTransform(base_frame_id, sensor_frame_id, ros::Time(0));
        const Eigen::Matrix4d mat = tf2::transformToEigen(transform.transform).matrix().cast<double>();
        std::cout << "      cloud trans from ["<<sensor_frame_id<<"] to ["<<base_frame_id<<"],  Mat:\n" << mat << std::endl;
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, mat);
        cloud_ptr->header.frame_id = base_frame_id;

    }catch(tf2::TransformException& ex){
        std::cout << ex.what() << std::endl;
        return;
    }

    // transform occupancy grid map
    const Eigen::Vector2d odom_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y);
    const double yaw = tf2::getYaw(msg_odom->pose.pose.orientation);

    // 计算在diff_odom的x与y分量在上一次旋转yaw方向上的x与y分量
    const Eigen::Vector2d diff_odom = Eigen::Rotation2Dd(-last_yaw).toRotationMatrix() * (odom_position - last_odom_position);
    double diff_yaw = yaw - last_yaw;
    diff_yaw = atan2(sin(diff_yaw), cos(diff_yaw)); //归一化，使处于-π~π。

    std::cout << "      diff odom:  [" << diff_odom(0) <<", "<<diff_odom(1)<<"] "<< std::endl;
    std::cout << "      diff yaw: " << diff_yaw << std::endl;

    transform_occupancy_grid_map(-diff_odom, -diff_yaw, occupancy_grid_map_);//更新ogm占有率地图，上帧地图+odom/tf=本帧地图

    input_cloud_to_occupancy_grid_map(cloud_ptr);// 对ogm地图，根据实时点云，进行占有率更新

    publish_occupancy_grid_map(msg_odom->header.stamp, base_frame_id);//将ogm占有率地图发布为ros格式：nav_msgs::OccupancyGrid

    CloudXYZIPtr dynamic_cloud_ptr(new CloudXYZI);
    dynamic_cloud_ptr->header = cloud_ptr->header;
    CloudXYZIPtr static_cloud_ptr(new CloudXYZI);
    static_cloud_ptr->header = cloud_ptr->header;

    devide_cloud(cloud_ptr, dynamic_cloud_ptr, static_cloud_ptr);

    // down_size_filter(static_cloud_ptr);

    sensor_msgs::PointCloud2Ptr msg_dynamic_cloud_ptr = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2);  
    sensor_msgs::PointCloud2Ptr msg_static_cloud_ptr = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2);  

    pcl::toROSMsg(*dynamic_cloud_ptr, *msg_dynamic_cloud_ptr);
    pcl::toROSMsg(*static_cloud_ptr, *msg_static_cloud_ptr);

    msg_dynamic_cloud_ptr->header = pcl_conversions::fromPCL(dynamic_cloud_ptr->header);
    msg_dynamic_cloud_ptr->header.frame_id = "/rslidar_link"; //动态点 坐标为雷达坐标系，方便观看动态物体在实车周围的情况
    msg_static_cloud_ptr->header = pcl_conversions::fromPCL(static_cloud_ptr->header);
    // msg_static_cloud_ptr->header.frame_id = "/A_LO_lidar_init"; 
    msg_static_cloud_ptr->header.frame_id = "/rslidar_link"; // 静态点 坐标为雷达坐标系
    
    static int delay_time_now = 0;
    if (delay_time_now > DELAYTIMES2PUB)
    {
        dynamic_pub_.publish(msg_dynamic_cloud_ptr);
        static_pub_.publish(msg_static_cloud_ptr);
    }
    else{
        delay_time_now++;
    }

    last_odom_position = odom_position;
    last_yaw = yaw;

    ROS_INFO("       Pub Static Points Num = %zd", msg_static_cloud_ptr->data.size());
    std::cout << "      cost time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
    std::cout << "********************Epoch Is Over *******************\n\n" << std::endl;

}

void DynamicFilter::callback_do_murge_ground(const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud, 
                                                            const sensor_msgs::PointCloud2ConstPtr& msg_obstacles_cloud_ground,
                                                            const nav_msgs::OdometryConstPtr& msg_odom)
{
    if (!doMurgeTheGroundPoints_) // 在这里而非在“回调的声明”进行if判断，是因为在声明处判断会导致“声明失效”，无法进入回调 - ljx0411测试
        return;

    std::cout << "********************Epoch Is Begin *******************" << std::endl;

    // ljx理解备注：
    // 使用odom话题数据，是为了让栅格地图trans，应该使用map->veh的实际odom
    // 使用tf广播数据，是为了让点云trans，应该使用veh->lidar的实际tf
    const double start_time = ros::Time::now().toSec();
    static Eigen::Vector2d last_odom_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y);
    static double last_yaw = tf2::getYaw(msg_odom->pose.pose.orientation);

    std::cout << "--------[0]  DynamicFilter Callback --------" << std::endl;
    CloudXYZIPtr cloud_ptr(new CloudXYZI);
    CloudXYZIPtr cloud_ground_ptr(new CloudXYZI);
    pcl::fromROSMsg(*msg_obstacles_cloud, *cloud_ptr);
    pcl::fromROSMsg(*msg_obstacles_cloud_ground, *cloud_ground_ptr);

    std::cout << "      received cloud without ground size:     " << cloud_ptr->points.size() << std::endl;
    std::cout << "      received cloud ground size:     " << cloud_ground_ptr->points.size() << std::endl;

    // 原始点云坐标系, 雷达的坐标系(与/rslidar_link等价的坐标系)
    const std::string sensor_frame_id = remove_first_slash(msg_obstacles_cloud->header.frame_id);
    std::cout << "      [L] sensor_frame_id:     " << sensor_frame_id << std::endl;

    // 订阅的地面点的坐标系（与上述雷达坐标系分开是为了解耦防止后续应用有误）
    const std::string sensor_ground_frame_id = remove_first_slash(msg_obstacles_cloud_ground->header.frame_id);
    std::cout << "      [G] sensor_ground_frame_id:     " << sensor_ground_frame_id << std::endl;

    // 目标坐标系，车体坐标系(与/veh等价的坐标系)
    const std::string base_frame_id = remove_first_slash(msg_odom->child_frame_id); 
    std::cout << "      [V] base_frame_id:     " << base_frame_id << std::endl; //如果frame不对，检查上一行是child_frame_id, 还是header.frame_id


    // 将雷达实时点云，从sensor_frame（雷达frame）转到base_frame（车体frame）下
    try{
        geometry_msgs::TransformStamped transform;
        Eigen::Matrix4d mat;

        transform = tf_buffer_.lookupTransform(base_frame_id, sensor_frame_id, ros::Time(0));
        mat = tf2::transformToEigen(transform.transform).matrix().cast<double>();
        std::cout << "      cloud trans from ["<<sensor_frame_id<<"] to ["<<base_frame_id<<"],  Mat:\n" << mat << std::endl;
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, mat);
        cloud_ptr->header.frame_id = base_frame_id;

        transform = tf_buffer_.lookupTransform(base_frame_id, sensor_ground_frame_id, ros::Time(0));
        mat = tf2::transformToEigen(transform.transform).matrix().cast<double>();
        std::cout << "      cloud_ground trans from ["<<sensor_ground_frame_id<<"] to ["<<base_frame_id<<"],  Mat:\n" << mat << std::endl;
        pcl::transformPointCloud(*cloud_ground_ptr, *cloud_ground_ptr, mat);
        cloud_ground_ptr->header.frame_id = base_frame_id;

    }catch(tf2::TransformException& ex){
        std::cout << ex.what() << std::endl;
        return;
    }

    // transform occupancy grid map
    const Eigen::Vector2d odom_position(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y);
    const double yaw = tf2::getYaw(msg_odom->pose.pose.orientation);

    // 计算在diff_odom的x与y分量在上一次旋转yaw方向上的x与y分量
    const Eigen::Vector2d diff_odom = Eigen::Rotation2Dd(-last_yaw).toRotationMatrix() * (odom_position - last_odom_position);
    double diff_yaw = yaw - last_yaw;
    diff_yaw = atan2(sin(diff_yaw), cos(diff_yaw)); //归一化，使处于-π~π。

    std::cout << "      diff odom:  [" << diff_odom(0) <<", "<<diff_odom(1)<<"] "<< std::endl;
    std::cout << "      diff yaw: " << diff_yaw << std::endl;

    transform_occupancy_grid_map(-diff_odom, -diff_yaw, occupancy_grid_map_);//更新ogm占有率地图，上帧地图+odom/tf=本帧地图

    input_cloud_to_occupancy_grid_map(cloud_ptr);// 对ogm地图，根据实时点云，进行占有率更新

    publish_occupancy_grid_map(msg_odom->header.stamp, base_frame_id);//将ogm占有率地图发布为ros格式：nav_msgs::OccupancyGrid

    CloudXYZIPtr dynamic_cloud_ptr(new CloudXYZI);
    CloudXYZIPtr static_cloud_ptr(new CloudXYZI);

    devide_cloud(cloud_ptr, dynamic_cloud_ptr, static_cloud_ptr);

    // 增加 [地面点] 到 [静态点] 中
    *static_cloud_ptr = *cloud_ground_ptr + *static_cloud_ptr;
    
    static_cloud_ptr->header = cloud_ptr->header;
    dynamic_cloud_ptr->header = cloud_ptr->header;

    // down_size_filter(static_cloud_ptr);

    sensor_msgs::PointCloud2Ptr msg_dynamic_cloud_ptr = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2);  
    sensor_msgs::PointCloud2Ptr msg_static_cloud_ptr = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2);  

    pcl::toROSMsg(*dynamic_cloud_ptr, *msg_dynamic_cloud_ptr);
    pcl::toROSMsg(*static_cloud_ptr, *msg_static_cloud_ptr);

    msg_dynamic_cloud_ptr->header = pcl_conversions::fromPCL(dynamic_cloud_ptr->header);
    msg_dynamic_cloud_ptr->header.frame_id = "/rslidar_link"; //动态点 坐标为雷达坐标系，方便观看动态物体在实车周围的情况
    msg_static_cloud_ptr->header = pcl_conversions::fromPCL(static_cloud_ptr->header);
    // msg_static_cloud_ptr->header.frame_id = "/A_LO_lidar_init"; 
    msg_static_cloud_ptr->header.frame_id = "/rslidar_link"; // 静态点 坐标为雷达坐标系
    
    static int delay_time_now_do_murge = 0;
    if (delay_time_now_do_murge > DELAYTIMES2PUB)
    {
        dynamic_pub_.publish(msg_dynamic_cloud_ptr);
        static_pub_.publish(msg_static_cloud_ptr);
    }
    else{
        delay_time_now_do_murge++;
    }


    last_odom_position = odom_position;
    last_yaw = yaw;

    ROS_INFO("       Pub Static Points Num = %zd", msg_static_cloud_ptr->data.size());
    std::cout << "      cost time: " << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
    std::cout << "********************Epoch Is Over *******************\n\n" << std::endl;

}

void DynamicFilter::transform_occupancy_grid_map(const Eigen::Vector2d& translation, double diff_yaw, OccupancyGridMap& map)
{
    std::cout << "--------[1]  Transform Occupancy Grid Map --------" << std::endl;

    const double dx = translation(0);
    const double dy = translation(1);
    const double c_yaw = cos(diff_yaw);
    const double s_yaw = sin(diff_yaw);

    const double dx_grid = dx / resolution_; //x方向的实际物理位移，在栅格地图上x方向移动了几个栅格
    const double dy_grid = dy / resolution_;

    Eigen::Matrix3d affine; // 2维的旋转平移矩阵，应该是从本帧到上帧（往过去方向）
    affine << c_yaw, -s_yaw, dx_grid,
              s_yaw,  c_yaw, dy_grid,
                  0,      0,        1;
    std::cout << "      forward affine:\n" << affine << std::endl;
    const Eigen::Matrix3d affine_inverse = affine.inverse(); // 2维的旋转平移矩阵，应该是从上帧到本帧（往未来方向）

    OccupancyGridMap ogm(grid_num_);

    //遍历每一个map点，并映射到ogm点，完成后，即可得到利用odom变换后的每一个点在上帧地图（map）中的占有率，并存入新的地图（ogm）
    for(int i=0;i<grid_num_;i++)
    {
        const double x_i = get_x_index_from_index(i) - grid_width_2_; //将总栅格地图的中心定为(0, 0)，计算第i个栅格的坐标
        const double y_i = get_y_index_from_index(i) - grid_width_2_;
        Eigen::Vector3d ogm_i(x_i, y_i, 1); //假定的本帧点
        Eigen::Vector3d map_i = affine_inverse * ogm_i; //本帧点经逆变换后，在上帧的具体坐标（可能不是整数栅格位置）

        // 双线性插值
        //以该本帧点在上帧的对应位置（大概率不是整数点）为中心，在上帧地图中，往外找到4个最近的整数点，即1个非整数栅格点 转换为 4个整数栅格点
        const int x_0 = std::floor(map_i(0)); 
        const int x_1 = x_0 + 1;
        const int y_0 = std::floor(map_i(1)); //大小关系（例子）：y_0=4,   map_i(1)=4.3,   y_1=5
        const int y_1 = y_0 + 1;

        if(x_0 < -grid_width_2_ || grid_width_2_ <= x_1) //本帧点变换后，若超地图边界则不要
            continue;
        if(y_0 < -grid_width_2_ || grid_width_2_ <= y_1)
            continue;
        
        // 计算这4个点在一条线上的index，(括号里的内容：将栅格地图原点从中心位置迁移至第三象限顶点↓)
        const int index_0_0 = (y_0 + grid_width_2_) * grid_width_ + (x_0 + grid_width_2_);
        const int index_0_1 = (y_1 + grid_width_2_) * grid_width_ +( x_0 + grid_width_2_);
        const int index_1_0 = (y_0 + grid_width_2_) * grid_width_ + (x_1 + grid_width_2_);
        const int index_1_1 = (y_1 + grid_width_2_) * grid_width_ + (x_1 + grid_width_2_);

        const Eigen::Vector2d y_vec(y_1 - map_i(1), map_i(1) - y_0);// 算出上述4个整数栅格点，到实际非整数栅格点的差值（小于1.0）
        const Eigen::Vector2d x_vec(x_1 - map_i(0), map_i(0) - x_0);
        Eigen::Matrix2d value_mat; // 这4个整数栅格点在上帧地图中的占有率
        
        value_mat << map[index_0_0].get_occupancy(), map[index_1_0].get_occupancy(),
                                     map[index_0_1].get_occupancy(), map[index_1_1].get_occupancy();

        // 根据实际非整数栅格点与4个整数栅格点的距离，作为权重，
        // 与4个整数栅格点在上帧的占有率做加权求和，作为该非整数栅格点的实际上一帧占有率，亦即“双线性差值”
        const double ogm_value = y_vec.transpose() * value_mat * x_vec;
        ogm[i].log_odds = std::log(ogm_value / (1 - ogm_value)); // ln(x/(1-x))

    }
    map.clear();
    map = ogm;
}

void DynamicFilter::input_cloud_to_occupancy_grid_map(const CloudXYZIPtr& cloud_ptr)
{
    std::cout << "--------[2] Input Cloud to Occupancy Grid Map --------" << std::endl;
    
    // 光束容器，存放每一个光束段内，与原点最近的点云的 最近距离
    std::vector<double>beam_list(beam_num_, sqrt(2) * width_2_);
    const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num_; //每一个光束分配到的弧度

    const int cloud_size = cloud_ptr->points.size();
    std::vector<bool> obstacle_indices(grid_num_, false);  //有点云在的栅格，即是障碍物栅格，该vector值设为true
    for(int i=0;i<cloud_size;i++){
        const auto& p = cloud_ptr->points[i];
        if(!is_valid_point(p.x, p.y)){
            continue;
        }

        // 以base_frame为坐标系/原点，计算每个点的距离、角度、所在光束index
        const double distance = sqrt(p.x * p.x + p.y * p.y); 
        const double direction = atan2(p.y, p.x);
        const int beam_index = (direction + M_PI) / beam_angle_resolution;

        // 计算该index的光束范围内最近点的 最近距离
        if(0 <= beam_index && beam_index < beam_num_){
            beam_list[beam_index] = std::min(beam_list[beam_index], distance);
        }

        //该点在总栅格地图中的的位置索引（第几个小栅格）
        const int index = get_index_from_xy(p.x, p.y); 
        if(index < 0 || grid_num_ <= index){
            continue;
        }
        obstacle_indices[get_index_from_xy(p.x, p.y)] = true;
    }

    // 根据实时点云信息，更新ogm地图的占有率，某栅格点云越多，占有率越高
    for(int i=0;i<grid_num_;i++){
        if(obstacle_indices[i]){
            occupancy_grid_map_[i].add_log_odds(log_odds_increase_);
        }
    }

    // 对每一个beam光束，其最近点往里的范围的栅格，如果全局下该栅格本帧无点云，则占有率减小（增强对比度）
    set_clear_grid_cells(beam_list, obstacle_indices, occupancy_grid_map_);
}

void DynamicFilter::publish_occupancy_grid_map(const ros::Time& stamp, const std::string& frame_id)
{
    std::cout << "--------[3] Publish Occupancy Grid Map --------" << std::endl;
    nav_msgs::OccupancyGrid og;
    og.header.stamp = stamp;
    og.header.frame_id = frame_id;
    og.info.resolution = resolution_;
    og.info.width = grid_width_;
    og.info.height = grid_width_;
    og.info.origin.position.x = -width_2_; //保证地图中央为原点（0, 0）
    og.info.origin.position.y = -width_2_;
    og.info.origin.position.z = -1.5; //XUV25B  最低悬挂时，middle离地高度1.5m
    og.info.origin.orientation.w = 1.0;
    og.data.resize(grid_num_);
    for(int i=0;i<grid_num_;i++){
        og.data[i] = occupancy_grid_map_[i].get_occupancy() * 100;
    }
    std::cout << "      [M] local_map_frame_id:     " << og.header.frame_id << std::endl;

    grid_pub_.publish(og);
}

void DynamicFilter::devide_cloud(const CloudXYZIPtr& cloud, CloudXYZIPtr& dynamic_cloud, CloudXYZIPtr& static_cloud)
{
    std::cout << "--------[4]  Devide Cloud to Dynamic & Static --------" << std::endl;

    dynamic_cloud->points.clear();
    static_cloud->points.clear();

    for(const auto& pt : cloud->points){

        if(-width_2_ <= pt.x && pt.x <= width_2_ && -width_2_ <= pt.y && pt.y <= width_2_){
            const int index = get_index_from_xy(pt.x, pt.y);

            if(0 <= index && index < grid_num_){
                const double occupancy = occupancy_grid_map_[index].get_occupancy();

                if(occupancy < occupancy_threshold_){ // 占有率小，定义为动态点
                    dynamic_cloud->points.push_back(pt);
                }
                else{
                    static_cloud->points.push_back(pt); //占有率大，定义为静态点
                }
            }
        }
    }

    std::cout << "      Dynamic Points' Number In This Epoch:    [" << dynamic_cloud->size() <<"] "<< std::endl;
    std::cout << "      Static Points' Number In This Epoch:     [" << static_cloud->size() <<"] "<< std::endl;

}

void DynamicFilter::down_size_filter(CloudXYZIPtr& static_cloud)
{
    std::cout << "--------[5]  Down Size Filter Of Static_Cloud --------" << std::endl;

    downSizeFilterStatic.setInputCloud(static_cloud);
    downSizeFilterStatic.filter(*static_cloud);
}

std::string DynamicFilter::remove_first_slash(std::string frame_id)
{
    const int slash_pos = frame_id.find('/');
    if(slash_pos == 0){
        frame_id.erase(0, 1);
    }
    return frame_id;
}

void DynamicFilter::set_clear_grid_cells(const std::vector<double>& beam_list, const std::vector<bool>& obstacle_indices, 
                                                                                    OccupancyGridMap& map)
{
    std::vector<bool> clear_indices(grid_num_, false);
    const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num_;

    // 处理每一个光束
    for(int i=0;i<beam_num_;i++){
        double direction = i * beam_angle_resolution - M_PI; 
        direction = atan2(sin(direction), cos(direction)); //角度归一化
        const double c = cos(direction);
        const double s = sin(direction);

        // 在该光束范围内，从原点->光束段内最近点之间的栅格，若合法且不是障碍物点，则可以清除clear
        for(double range=0.0; range<beam_list[i]; range+=resolution_){
            const double x = range * c;
            const double y = range * s;
            if(is_valid_point(x, y)){
                const int index = get_index_from_xy(x, y);
                if(!obstacle_indices[index]){
                    clear_indices[index] = true;
                }
                else{
                    break;
                }
            }
            else{
                break;
            }
        }
    }

    for(int i=0;i<grid_num_;++i){
        if(clear_indices[i]){
            map[i].add_log_odds(-log_odds_decrease_);
        }
    }
}

double DynamicFilter::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - grid_width_2_) * resolution_;
}

double DynamicFilter::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - grid_width_2_) * resolution_;
}

void DynamicFilter::start_process(void)
{
    ros::spin();
}
