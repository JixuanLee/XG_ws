// 简介：NDT是一种基于正态分布变换的点云配准算法，它可以用于在已知的点云地图中进行激光雷达的定位
// 修订：2023.12.08 - - - - 适配徐工XUV25B
// 原理：
//              （1）NDT算法的基本思想是将参考点云（即已建好的点云地图）划分为许多小的体素（voxel），并计算每个体素内的点云的均值和协方差矩阵，
//              从而得到每个体素的正态分布模型。
//              （2）然后，NDT算法将输入点云（即实时扫描到的点云）与参考点云进行配准，通过优化一个目标函数来求解最佳的刚体变换矩阵，
//              使得输入点云变换后与参考点云的正态分布模型的匹配程度最高。
//              （3）目标函数的定义是基于输入点云中每个点在参考点云中对应的体素的正态分布概率密度的累加，也就是说，
//              如果输入点云变换后与参考点云的匹配很好，那么每个点在参考点云中的概率密度都会很大，从而目标函数的值也会很大。
//              （4）为了优化目标函数，NDT算法使用了牛顿法或者类似的线搜索方法，通过计算目标函数的梯度和海森矩阵来迭代更新刚体变换矩阵，
//              直到满足收敛条件或者达到最大迭代次数。

#include "ndt_localizer/ndt_localizer_core.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
:nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_)
{
    key_value_stdmap_["state"] = "Initializing";
    init_params();

    // Publishers
    ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_, 10); //NDT输出pose信息，坐标系：odom，时间戳：输入点云时间戳

    // Subscribers
    initial_pose_sub_ = nh_.subscribe(rviz_pose_topic_, 100, &NdtLocalizer::callback_init_pose, this);  
    map_points_sub_ = nh_.subscribe(map_points_topic_, 1, &NdtLocalizer::callback_pointsmap, this);
    sensor_points_sub_ = nh_.subscribe(current_points_topic_, 1, &NdtLocalizer::callback_pointcurrent, this);
    if (use_gnss_)  // XG合同要求：定位可手动切换是否启动GPS参与定位
        gnss_sub_ = nh_.subscribe(gnss_topic_, 10, &NdtLocalizer::callback_gnss, this);

    gnss_update_time_ = 100;  //gnss 20hz
    gnss_update_time_now_ = 0;
}

NdtLocalizer::~NdtLocalizer()
{
}

void NdtLocalizer::init_params()
{
    private_nh_.getParam("rviz_pose_topic", rviz_pose_topic_);
    private_nh_.getParam("current_points_topic", current_points_topic_);
    private_nh_.getParam("map_points_topic", map_points_topic_);
    private_nh_.getParam("gnss_topic", gnss_topic_);
    private_nh_.getParam("ndt_pose_topic", ndt_pose_topic_);
    private_nh_.getParam("use_gnss", use_gnss_);

    double trans_epsilon = ndt_.getTransformationEpsilon(); // 收敛阈值
    double step_size = ndt_.getStepSize(); // 步长
    double resolution = ndt_.getResolution(); // 分辩率
    int max_iterations = ndt_.getMaximumIterations(); // 最大迭代次数

    private_nh_.getParam("trans_epsilon", trans_epsilon);
    private_nh_.getParam("step_size", step_size);
    private_nh_.getParam("resolution", resolution);
    private_nh_.getParam("max_iterations", max_iterations);

    veh_frame_ = "veh";
    map_frame_ = "map";

    ndt_.setTransformationEpsilon(trans_epsilon);
    ndt_.setStepSize(step_size);
    ndt_.setResolution(resolution);
    ndt_.setMaximumIterations(max_iterations);

    ROS_INFO("trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", 
             trans_epsilon, step_size, resolution, max_iterations);

    private_nh_.getParam("converged_param_transform_probability", converged_param_transform_probability_); // 收敛参数可靠性阈值
}

//  滤除圆形区域外的点
void NdtLocalizer::removePointsByRange(pcl::PointCloud<pcl::PointXYZ> in_pcl, pcl::PointCloud<pcl::PointXYZ> out_pcl, double min_range, double max_range)
{
    if(min_range >= max_range) 
    {
        ROS_ERROR_ONCE("min_range>=max_range @(%lf, %lf)", min_range, max_range );
        return;
    }

    double square_min_range = min_range * min_range;
    double square_max_range = max_range * max_range;

    out_pcl.header = in_pcl.header;
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = in_pcl.begin(); iter != in_pcl.end(); ++iter)
    {
        const pcl::PointXYZ &p = *iter;
        double square_distance = p.x * p.x + p.y * p.y;
        if(square_min_range <= square_distance && square_distance <= square_max_range)
        out_pcl.points.push_back(p);
    }
}

void NdtLocalizer::downsampler(const sensor_msgs::PointCloud2::ConstPtr& in, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*in, *in_pcl_ptr);
    removePointsByRange(*in_pcl_ptr, *in_pcl_ptr, 0, MAX_MEASUREMENT_RANGE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_in_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 filtered_msg;

    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
    if (voxel_leaf_size >= 0.1)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(in_pcl_ptr);
        voxel_grid_filter.filter(*filtered_in_pcl_ptr);
        out = filtered_in_pcl_ptr;
    }
    else
    {
        out = in_pcl_ptr;
    }
}

void NdtLocalizer::callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr)
{
    std::cout<<"\033[32m"<<"[0] initPoseCallback Now."<<std::endl;

    if (initial_pose_msg_ptr->header.frame_id != "map"  &&  initial_pose_msg_ptr->header.frame_id != "/map")
        ROS_INFO("\033[33mNDT suggest you pub init pose in [map] frame by Rviz.");

    // 将Rviz发布的初始位姿对齐到map坐标系下
    if (initial_pose_msg_ptr->header.frame_id == map_frame_) 
        initial_pose_cov_msg_ = *initial_pose_msg_ptr;
    else
    {
        //get 从Rviz发布的Pose信息的坐标系 -> map坐标系 的tf
        geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
        get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

        // 将Rviz发布的Pose信息变换至map坐标系
        geometry_msgs::PoseWithCovarianceStamped::Ptr aftTF2Map_initial_pose_msg_ptr(new geometry_msgs::PoseWithCovarianceStamped);
        tf2::doTransform(*initial_pose_msg_ptr, *aftTF2Map_initial_pose_msg_ptr, *TF_pose_to_map_ptr);

        // aftTF2Map_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
        initial_pose_cov_msg_ = *aftTF2Map_initial_pose_msg_ptr;
    }
    pcl::PointXYZ origin_guess_point, nearest_guass_point;
    origin_guess_point.x = initial_pose_cov_msg_.pose.pose.position.x;
    origin_guess_point.y = initial_pose_cov_msg_.pose.pose.position.y;
    origin_guess_point.z = initial_pose_cov_msg_.pose.pose.position.z;
    nearest_guass_point = findClosestPointInMap(map_points_ptr, origin_guess_point);
    initial_pose_cov_msg_.pose.pose.position.z = nearest_guass_point.z;

    // if click the initpose again, re init！
    init_pose = false;
}

void NdtLocalizer::callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr)
{
    // 该回调只进入一次，因为订阅的话题只发布一次
    std::cout<<"\033[32m"<<"[2] pointsMapCallback Now."<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);

    const auto trans_epsilon = ndt_.getTransformationEpsilon();
    const auto step_size = ndt_.getStepSize();
    const auto resolution = ndt_.getResolution();
    const auto max_iterations = ndt_.getMaximumIterations();

    // ndt_new：只是为了更新ndt_对象的目标点云target，并初始化变换矩阵，随后赋值给ndt_，不执行实际的配准过程。
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

    ndt_new.setTransformationEpsilon(trans_epsilon);
    ndt_new.setStepSize(step_size);
    ndt_new.setResolution(resolution);
    ndt_new.setMaximumIterations(max_iterations);

    ndt_new.setInputTarget(map_points_ptr); // PCD地图的点云作为target

    ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());

    // swap
    ndt_map_mtx_.lock();
    ndt_ = ndt_new;
    ndt_map_mtx_.unlock();
}

void NdtLocalizer::callback_pointcurrent(const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr)
{
    std::cout<<"\033[32m"<<"[3] pointsCurrentCallback Now."<<std::endl;
    const auto exe_start_time = std::chrono::system_clock::now();


    // mutex Map
    // 当lock_guard对象被创建时，它会尝试获取互斥锁的所有权，
    // 如果互斥锁已经被另一个线程锁定，那么创建lock_guard对象的线程将会阻塞，直到互斥锁被释放。
    // 当lock_guard对象离开其作用域时，它会自动释放互斥锁。
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id; // rslidar_link
    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp; //点云bag包/实时front雷达数据时间戳
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    downsampler(sensor_points_sensorTF_msg_ptr, sensor_points_sensorTF_ptr);

    // get 从veh -> rslidar_link 的tf
    geometry_msgs::TransformStamped::Ptr TF_veh_to_sensor_ptr(new geometry_msgs::TransformStamped);
    get_transform(veh_frame_, sensor_frame, TF_veh_to_sensor_ptr); //veh->rslidar_link

    const Eigen::Affine3d veh_to_sensor_affine = tf2::transformToEigen(*TF_veh_to_sensor_ptr);
    const Eigen::Matrix4f veh_to_sensor_matrix = veh_to_sensor_affine.matrix().cast<float>();

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_vehTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sensor_points_sensorTF_ptr, *sensor_points_vehTF_ptr, veh_to_sensor_matrix);
    
    std::cout<<"MapPoints:"<<map_points_ptr->size()<<"NowPoints:"<<sensor_points_vehTF_ptr->size()<<std::endl;
    
    // set input point cloud
    ndt_.setInputSource(sensor_points_vehTF_ptr); //veh坐标系下的实时点云作为source

    if (ndt_.getInputTarget() == nullptr) 
    {
        ROS_WARN_STREAM_THROTTLE(1, "NDT's Target Points is NULL!");
        return;
    }

    // align配准
    Eigen::Matrix4f initial_pose_matrix;

    if (!init_pose)
    { 
        // 处理来自rviz中的点，进行init
        Eigen::Affine3d initial_pose_affine;
        tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
        initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
        
        // for the first time, we don't know the pre_trans, so just use the init_trans, 
        // which means, the delta trans for the second time is 0
        pre_trans = initial_pose_matrix;
        // init_pose = true;
    }
    else
    {
        // use predicted pose as init guess (currently we only impl linear model)
        // 下方的pre是上帧配准的结果，delta是上帧配准的结果与上上帧配准结果的差距
        // 即：第0帧的初始估计位姿 = 第-1帧配准结果 * 第-1帧与第-2帧配准结果的差距
        // 这里，使用了“第-1帧与第-2帧配准结果的差距”作为“第0帧与第-1帧配准结果的差距”的线性估算值
        initial_pose_matrix = pre_trans * delta_trans;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();
    key_value_stdmap_["state"] = "Aligning";

    // 执行点云配准操作，利用NDT算法来对齐source、target两个点云数据，直到收敛或达最大迭代次数，最终的变换矩阵和配准得分保存在类对象成员变量里
    ndt_.align(*output_cloud, initial_pose_matrix); // 参数1保存了source点云变换后的结果，也就是与target点云配准后的点云；参数2是初始猜想矩阵。

    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time = std::chrono::system_clock::now();
    const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;

    const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

    // score: NDT算法得分，通过黑盒子算法，体现目标点云与源点云之间的匹配程度，trans_probability_ = score / NDT源点云点的数量
    // transform_probability: 基于高斯分布的概率密度函数，计算源点云中的每个点与目标点云之间的匹配概率，并将这些概率相加得到最终的转移概率
    const float transform_probability = ndt_.getTransformationProbability();
    const int iteration_num = ndt_.getFinalNumIteration();

    bool is_converged = true;
    static size_t skipping_publish_num = 0;

    // 实际迭代过多、转换概率过低，即判定“不收敛”
    // 实测，定位效果较好时，迭代次数一般在10以下，基本上都收敛，transform_probability基本上在4以上
    if (iteration_num >= ndt_.getMaximumIterations() + 2 ||
        transform_probability < converged_param_transform_probability_) 
    {
        is_converged = false;
        ++skipping_publish_num;
        std::cout<<"\033[33m"<<"    NDT配准发散, 点云重定位 失败! 尝试重新下发Rviz初始位姿 或 调节NDT参数。"<<std::endl;
    } 
    else 
    {
        skipping_publish_num = 0;
        std::cout<<"\033[32m"<<"    NDT配准收敛, 点云重定位 成功! "<<std::endl;
    }

    // 计算本帧与上帧配准结果的差距delta，作为下帧与本帧配准结果差距的估计值
    delta_trans = pre_trans.inverse() * result_pose_matrix;

    // std::cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<" z: "<<delta_translation(2)<<std::endl;
    // std::cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<" roll: "<<delta_euler(2)<<std::endl;

    pre_trans = result_pose_matrix;
    
    // publish
    geometry_msgs::PoseStamped result_pose_stamped_msg;
    result_pose_stamped_msg.header.stamp = sensor_ros_time;
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_msg.pose = result_pose_msg;

    if (is_converged)
        ndt_pose_pub_.publish(result_pose_stamped_msg);

    try
    {
        geometry_msgs::PoseStamped odom_to_veh_pose;
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = sensor_ros_time;
        tfs.header.frame_id = "odom";
        tfs.child_frame_id = veh_frame_;
        odom_to_veh_pose = tf2_buffer_.transform(result_pose_stamped_msg, "odom"); 
        // tfs.transform.translation.x = odom_to_veh_pose.pose.position.x;
        // tfs.transform.translation.y = odom_to_veh_pose.pose.position.y;
        // tfs.transform.translation.z = odom_to_veh_pose.pose.position.z;
        // tfs.transform.rotation = odom_to_veh_pose.pose.orientation;
        
        tfs.transform.translation.x = odom_to_veh_pose.pose.position.x;
        tfs.transform.translation.y = odom_to_veh_pose.pose.position.y;
        tfs.transform.translation.z = odom_to_veh_pose.pose.position.z;
        tfs.transform.rotation = odom_to_veh_pose.pose.orientation;

        tf2_broadcaster_.sendTransform(tfs); 
    }
    catch(const tf2::TransformException & ex)
    {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
    }
    
    if (!init_pose)
    {
        if (is_converged)
        {
            gnss_bias_[0].push_back(initial_pose_cov_msg_.pose.pose.position.x - result_pose_stamped_msg.pose.position.x);
            gnss_bias_[1].push_back(initial_pose_cov_msg_.pose.pose.position.y - result_pose_stamped_msg.pose.position.y);
            gnss_bias_[2].push_back(initial_pose_cov_msg_.pose.pose.position.z - result_pose_stamped_msg.pose.position.z);

            if (gnss_bias_[0].size() > 5)
            {
                gnss_bias_[0].pop_front();
                gnss_bias_[1].pop_front();
                gnss_bias_[2].pop_front();
            }

            for (int i = 0; i < gnss_bias_[0].size(); i++)
            {
                gnss_bias_average_[0] += gnss_bias_[0][i];
                gnss_bias_average_[1] += gnss_bias_[1][i];
                gnss_bias_average_[2] += gnss_bias_[2][i];
            }

            gnss_bias_average_[0] /= gnss_bias_[0].size();
            gnss_bias_average_[1] /= gnss_bias_[0].size();
            gnss_bias_average_[2] /= gnss_bias_[0].size();
        }

        init_pose = true;
    }

    key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
    key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
    key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "    align_time(配准耗时): " << align_time << "ms" << std::endl;
    std::cout << "    exe_time(总计耗时): " << exe_time << "ms" << std::endl;
    std::cout << "    trans_prob(定位结果置信系数): " << transform_probability 
                        <<"    良好定位最低系数: "<<converged_param_transform_probability_<< std::endl;
    std::cout << "    iter_num(配准迭代次数): " << iteration_num << std::endl;
    std::cout << "    skipping_publish_num(持续配准失败总帧数): " << skipping_publish_num << std::endl;
}

void NdtLocalizer::callback_gnss(const nav_msgs::Odometry & msg)
{
    static bool init = false;
    if (init == true)
    {
        // gnss_update_time_now_++;
        // if (gnss_update_time_now_ <= gnss_update_time_)
        //     return;
        // else
        // {
        //     gnss_update_time_now_ = 0;
            if (ndt_.getTransformationProbability() > converged_param_transform_probability_)
                return;
        // }
    }
    else
        init = true;
    
    ROS_WARN("NDT: low transform probability! re-localization by gnss!");
    // 每有 gnss_update_time_ 次数据，重新初始化定位，并更新偏差参数
    try
    {
        geometry_msgs::TransformStamped::Ptr TF_map_to_odom(new geometry_msgs::TransformStamped);
        get_transform(map_frame_, "odom", TF_map_to_odom);

        geometry_msgs::PoseStamped::Ptr veh_pose_in_odom(new geometry_msgs::PoseStamped);
        veh_pose_in_odom->header = msg.header;
        veh_pose_in_odom->pose = msg.pose.pose;

        geometry_msgs::PoseStamped::Ptr veh_pose_in_map(new geometry_msgs::PoseStamped);
        tf2::doTransform(*veh_pose_in_odom, *veh_pose_in_map, *TF_map_to_odom);

        initial_pose_cov_msg_.header.stamp = veh_pose_in_map->header.stamp;
        initial_pose_cov_msg_.pose.pose.position.x = veh_pose_in_map->pose.position.x - gnss_bias_average_[0];
        initial_pose_cov_msg_.pose.pose.position.y = veh_pose_in_map->pose.position.y - gnss_bias_average_[1];
        initial_pose_cov_msg_.pose.pose.position.z = veh_pose_in_map->pose.position.z - gnss_bias_average_[2];
        initial_pose_cov_msg_.pose.pose.orientation = veh_pose_in_map->pose.orientation;

        // 使用最近点搜索，使用地图中与当前位置（xy）最近的点的z值作为估计的z。
        // 这是为了解决XG2024-05-09发现的：SLAM建图的Z值落差大于实际情况，导致使用GPS的海拔数据作为Z值进行估计位姿提供给NDT是无效的，导致NDT无法进行定位收敛。
        // 该问题的表征：GPS给定的位姿会处于地图下方（因为SLAM地图的Z的尺度显著的变大了很多，GPS显示海拔变高13m，SLAM地图Z值落差达到60m）
        // 该问题的表征：在建图初始点附近，上述问题尚不严重（显然的，这个问题随着SLAM的进行而变严重），但再远些，会出现GPS给定位姿处于地图下方很远位置，该启发位姿就无效了。
        pcl::PointXYZ origin_guess_point, nearest_guass_point;
        origin_guess_point.x = initial_pose_cov_msg_.pose.pose.position.x;
        origin_guess_point.y = initial_pose_cov_msg_.pose.pose.position.y;
        origin_guess_point.z = initial_pose_cov_msg_.pose.pose.position.z;
        nearest_guass_point = findClosestPointInMap(map_points_ptr, origin_guess_point);
        initial_pose_cov_msg_.pose.pose.position.z = nearest_guass_point.z;

        init_pose = false;
    }        
    catch (tf2::TransformException & ex) 
    {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        ROS_ERROR_THROTTLE(1, "Please publish TF %s to %s", map_frame_.c_str(), veh_frame_.c_str());
    }
}

pcl::PointXYZ NdtLocalizer::findClosestPointInMap( const pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr, const pcl::PointXYZ& query_point)
{
    if(map_points_ptr->size() == 0)
    {
        std::cerr << "No map points data !!." << std::endl;
        return query_point;
    }

    kdtree.setInputCloud(map_points_ptr);

    int NumOfNear = 1;
    std::vector<int> pointIdxNKNSearch(NumOfNear);
    std::vector<float> pointNKNSquaredDistance(NumOfNear);

    // 执行最近邻搜索
    if (kdtree.nearestKSearch(query_point, NumOfNear, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
    {
        int indexNearset = pointIdxNKNSearch[0];
        pcl::PointXYZ pointNearest = map_points_ptr->points[indexNearset];

        std::cout << "Nearset Point: " << pointNearest << std::endl;
        return pointNearest;
    } 
    else 
    {
        std::cerr << "No nearest neighbor found." << std::endl;
        return query_point;
    }

}

bool NdtLocalizer::get_transform(const std::string & target_frame_id, const std::string & source_frame_id,
                                 const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr, const ros::Time & time_stamp)
{
    std::string target_frame = remove_first_slash(target_frame_id);
    std::string source_frame = remove_first_slash(source_frame_id); 

    if (target_frame == source_frame) {
        transform_stamped_ptr->header.stamp = time_stamp;
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return true;
    }

    try {
        *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
    } 
    catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

        transform_stamped_ptr->header.stamp = time_stamp;
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return false;
    }
    return true;
}

bool NdtLocalizer::get_transform(const std::string & target_frame_id, const std::string & source_frame_id,
                                 const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
    std::string target_frame = remove_first_slash(target_frame_id);
    std::string source_frame = remove_first_slash(source_frame_id); 

    if (target_frame == source_frame) {
        transform_stamped_ptr->header.stamp = ros::Time::now();
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return true;
    }

    try {
        *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    } 
    catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

        transform_stamped_ptr->header.stamp = ros::Time::now();
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return false;
    }
    return true;
}

std::string NdtLocalizer::remove_first_slash(std::string frame_id)
{
    const int slash_pos = frame_id.find('/');
    if(slash_pos == 0){
        frame_id.erase(0, 1);
    }
    return frame_id;
}


int main(int argc, char **argv)
{
    ROS_INFO("\033[1;32m---->\033[0m NDT Localizer Core Started.");

    ros::init(argc, argv, "ndt_localizer_core");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    NdtLocalizer ndt_localizer(nh, private_nh);

    ros::spin();
    return 0;
}