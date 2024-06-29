#include "lidar_preprocess/preprocess.h"


Preprocessor::Preprocessor() :  node_handle_(), private_node_handle_("~"), front_cloud_ptr_(new PointCloudT),
                                left_cloud_ptr_(new PointCloudT), right_cloud_ptr_(new PointCloudT), 
                                middle_cloud_ptr_(new PointCloudT), all_cloud_ptr_(new PointCloudT),
                                sync(syncPolicy(100))
{
    private_node_handle_.param("staticyaml_dir", staticyaml_dir_, std::string(""));
    ReadStaticYaml(staticyaml_dir_);
    private_node_handle_.param("isMultiLidar", isMultiLidar, false);
    // private_node_handle_.param("doGroundRemoval", doGroundRemoval, true); //默认为true，即默认滤除地面点

    inilerFilterWithoutGround.setRadiusSearch(0.3);                  // 设置搜索半径 0.5
    inilerFilterWithoutGround.setMinNeighborsInRadius(5);            // 设置一个内点最少的邻居数目 
    inilerFilterGround.setRadiusSearch(0.3);                  // 设置搜索半径 0.5
    inilerFilterGround.setMinNeighborsInRadius(5);            // 设置一个内点最少的邻居数目 

    size_L = 50;  //50
    size_W = 50;  //120
    size_cell = cell_size_ ;
    
    cell_isCover = new int*[int(2*size_L/size_cell)] ;
    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        cell_isCover[i] = new int[int(2*size_W/size_cell)];

    cell_max_Z = new float*[int(2*size_L/size_cell)] ;
    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        cell_max_Z[i] = new float[int(2*size_W/size_cell)];

    cell_min_Z = new float*[int(2*size_L/size_cell)] ;
    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        cell_min_Z[i] = new float[int(2*size_W/size_cell)];

    cell_not_ground = new int*[int(2*size_L/size_cell)] ;
    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        cell_not_ground[i] = new int[int(2*size_W/size_cell)];
}

Preprocessor::~Preprocessor()
{
    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        delete[] cell_isCover[i];
    delete[] cell_isCover ;

    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        delete[] cell_max_Z[i];
    delete[] cell_max_Z ;

    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        delete[] cell_min_Z[i];
    delete[] cell_min_Z ;

    for (size_t i = 0; i < int(2*size_L/size_cell); i++)
        delete[] cell_not_ground[i];
    delete[] cell_not_ground ;
}


void Preprocessor::Run()
{
    // ground_middle_pub_ = nh_.advertise<pcl::PCLPointCloud2>("/middle_lidar_pcl", 10); //PCL格式的点云
    // ground_front_pub_  = nh_.advertise<pcl::PCLPointCloud2>("/front_lidar_pcl", 10); 
    // ground_left_pub_   = nh_.advertise<pcl::PCLPointCloud2>("/left_lidar_pcl", 10);
    // ground_right_pub_  = nh_.advertise<pcl::PCLPointCloud2>("/right_lidar_pcl", 10);
    // points_concat_pub_ = nh_.advertise<pcl::PCLPointCloud2>("/concat_lidar_pcl", 10);

    if (isMultiLidar)
    {
        ros::Rate rate(10);

        ros_concat_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_concat_ros", 10); //总的拼接后的ros点云
        ros_cliped_all_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_preprocessed_ros", 10); //总的拼接、预处理后的有地面的ros点云
        ros_cliped_without_ground_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_preprocessed_without_ground_ros", 10); //总的拼接、预处理后的无地面的ros点云
        ros_ground_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_preprocessed_ground_ros", 10); //总的拼接、预处理后的地面的ros点云

        // ①分别接收4个topic
        mf_front_sub. subscribe(node_handle_, front_lidar_sub_topic_,  50, ros::TransportHints().tcpNoDelay());
        mf_left_sub.  subscribe(node_handle_, left_lidar_sub_topic_,   50, ros::TransportHints().tcpNoDelay());
        mf_right_sub. subscribe(node_handle_, right_lidar_sub_topic_,  50, ros::TransportHints().tcpNoDelay());
        mf_middle_sub.subscribe(node_handle_, middle_lidar_sub_topic_, 50, ros::TransportHints().tcpNoDelay());
        
        // ②将4个topic的数据进行同步
        sync.connectInput(mf_front_sub, mf_left_sub, mf_right_sub, mf_middle_sub);
        sync.registerCallback(boost::bind(&Preprocessor::MultiLidarCallback, this, _1, _2, _3, _4));

        while(ros::ok())
        {
            double start_time = ros::Time::now().toSec();
            ros::spinOnce();

            *all_cloud_ptr_ = *left_cloud_ptr_ + *right_cloud_ptr_;
            *all_cloud_ptr_ = *front_cloud_ptr_ + *all_cloud_ptr_;
            *all_cloud_ptr_ = *middle_cloud_ptr_ + *all_cloud_ptr_;
            all_cloud_ptr_->header.stamp = middle_cloud_ptr_->header.stamp;

            // TransformCloud2TargetLink(all_cloud_ptr_, T_middle2veh); //将激光雷达坐标系/rslida_link转到车体坐标系/veh
            // points_concat_pub_.publish(*all_cloud_ptr_);  
            TransPubPCL2ROS(all_cloud_ptr_, ros_concat_points_pub_);

            PointCloudT::Ptr cloud_cliped_all_ptr(new PointCloudT);
            PointCloudT::Ptr cloud_cliped_without_ground_ptr(new PointCloudT);
            PointCloudT::Ptr cloud_cliped_ground_ptr(new PointCloudT);

            ClipCloud(all_cloud_ptr_, cloud_cliped_all_ptr, cloud_cliped_without_ground_ptr, cloud_cliped_ground_ptr);

            TransPubPCL2ROS(cloud_cliped_all_ptr, ros_cliped_all_points_pub_);
            TransPubPCL2ROS(cloud_cliped_without_ground_ptr, ros_cliped_without_ground_points_pub_);
            TransPubPCL2ROS(cloud_cliped_ground_ptr, ros_ground_points_pub_);

            ROS_INFO("\033[32mLidar's State:   Front:  %d,   Left:  %d,   Right:  %d,   Middle:  %d", 
                    isLidarGotIn[0], isLidarGotIn[1], isLidarGotIn[2], isLidarGotIn[3]);
            isLidarGotIn[4] = {0};

            double end_time = ros::Time::now().toSec();
            ROS_INFO("once spin cost time: %f sec",end_time-start_time);
            rate.sleep();
            std::cout << "--------------------------once spin-----------------------------------" << std::endl;
        }
    }
    // 单个雷达
    else
    {
        ros_cliped_all_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_preprocessed_ros", 10); //总的拼接、预处理后的有地面的ros点云
        ros_cliped_without_ground_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_preprocessed_without_ground_ros", 10); //总的拼接、预处理后的无地面的ros点云
        ros_ground_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_lidars_preprocessed_ground_ros", 10); //总的拼接、预处理后的地面的ros点云

        points_middle_sub_ = node_handle_.subscribe(middle_lidar_sub_topic_, 10, &Preprocessor::SingleLidarCallback, this);

        ros::spin();
    }
}

void Preprocessor::MultiLidarCallback(const sensor_msgs::PointCloud2ConstPtr & front, const sensor_msgs::PointCloud2ConstPtr & left,
                                      const sensor_msgs::PointCloud2ConstPtr & right, const sensor_msgs::PointCloud2ConstPtr & middle)
{
    isLidarGotIn[0] = 1;
    LidarTimeNow[0] = front->header.stamp;
    PointCloudT::Ptr front_pcl(new PointCloudT);
    pcl::fromROSMsg(*front, *front_pcl);
    TransformCloud2TargetLink(front_pcl, T_front2middle_); //转到middle_lidar下
    front_pcl->header.frame_id = "rslidar_link";
    // ground_front_pub_.publish(front_pcl);
    front_cloud_ptr_->clear();
    *front_cloud_ptr_ = *front_pcl;

    isLidarGotIn[1] = 1;
    LidarTimeNow[1] = left->header.stamp;
    PointCloudT::Ptr left_pcl(new PointCloudT);
    pcl::fromROSMsg(*left, *left_pcl);
    TransformCloud2TargetLink(left_pcl, T_left2middle_); //转到middle_lidar下
    left_pcl->header.frame_id = "rslidar_link";
    // ground_left_pub_.publish(left_pcl);
    left_cloud_ptr_->clear();
    *left_cloud_ptr_ = *left_pcl;

    isLidarGotIn[2] = 1;
    LidarTimeNow[2] = right->header.stamp;
    PointCloudT::Ptr right_pcl(new PointCloudT);
    pcl::fromROSMsg(*right, *right_pcl);
    TransformCloud2TargetLink(right_pcl,T_right2middle_);
    right_pcl->header.frame_id = "rslidar_link";
    // ground_right_pub_.publish(right_pcl);
    right_cloud_ptr_->clear();
    *right_cloud_ptr_ = *right_pcl;

    isLidarGotIn[3] = 1;
    LidarTimeNow[3] = middle->header.stamp;
    PointCloudT::Ptr middle_pcl(new PointCloudT);
    pcl::fromROSMsg(*middle, *middle_pcl);
    middle_pcl->header.frame_id = "rslidar_link";
    // ground_middle_pub_.publish(middle_pcl);
    middle_cloud_ptr_->clear();
    *middle_cloud_ptr_ = *middle_pcl;

    double min_t = LidarTimeNow[0].toSec();
    double max_t = LidarTimeNow[0].toSec();
    for (int i = 1; i < 4; i++)
    {
        if (LidarTimeNow[i].toSec() < min_t)
            min_t = LidarTimeNow[i].toSec();
        if (LidarTimeNow[i].toSec() > max_t)
            max_t = LidarTimeNow[i].toSec();
    }

    ROS_INFO("\033[33mMax_Diff_Time is:  %f ms", (max_t - min_t)*1000 );
}

void Preprocessor::SingleLidarCallback(const sensor_msgs::PointCloud2ConstPtr & middle)
{
    PointCloudT::Ptr middle_pcl(new PointCloudT);
    pcl::fromROSMsg(*middle, *middle_pcl);
    middle_pcl->header.frame_id = "rslidar_link";
    middle_cloud_ptr_->clear();
    *middle_cloud_ptr_ = *middle_pcl;

    PointCloudT::Ptr cloud_cliped_all_ptr(new PointCloudT);
    PointCloudT::Ptr cloud_cliped_without_ground_ptr(new PointCloudT);
    PointCloudT::Ptr cloud_cliped_ground_ptr(new PointCloudT);

    ClipCloud(middle_cloud_ptr_, cloud_cliped_all_ptr, cloud_cliped_without_ground_ptr, cloud_cliped_ground_ptr);
    
    TransPubPCL2ROS(cloud_cliped_all_ptr, ros_cliped_all_points_pub_);
    TransPubPCL2ROS(cloud_cliped_without_ground_ptr, ros_cliped_without_ground_points_pub_);
    TransPubPCL2ROS(cloud_cliped_ground_ptr, ros_ground_points_pub_);
}

void Preprocessor::TransformCloud2TargetLink(PointCloudT::Ptr in_cloud, Eigen::Matrix4d Trans)
{
    Eigen::Matrix4d Trans_ = Trans;
    pcl::transformPointCloud(*in_cloud, *in_cloud, Trans_);
}

void Preprocessor::TransPubPCL2ROS(PointCloudT::Ptr &input_cloud, ros::Publisher ros_pub) 
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*input_cloud, output);
    output.header = pcl_conversions::fromPCL(input_cloud->header);
    output.header.frame_id = "/rslidar_link";
    ros_pub.publish(output);
}

void Preprocessor::ClipCloud(const PointCloudT::Ptr in_cloud_ptr, PointCloudT::Ptr out_all_cloud_ptr, 
                                                                          PointCloudT::Ptr out_without_ground_cloud_ptr, PointCloudT::Ptr out_ground_cloud_ptr)
{
    out_all_cloud_ptr->points.clear();
    out_without_ground_cloud_ptr->points.clear();
    out_ground_cloud_ptr->points.clear();

    ROS_INFO("preprocess_core.cpp: ClipCloud will Start! ");

    // 初始化
    for (unsigned int i = 0; i < int(2*size_L/size_cell); i++)
    { 
        for (unsigned int j = 0; j < int(2*size_W/size_cell); j++)
        {
            cell_isCover[i][j] = 0;
            cell_max_Z[i][j] = -999.0;
            cell_min_Z[i][j] = 999.0;
            cell_not_ground[i][j] = 0;
        }
    }

    //得到原点周围每一个xy处，z的max与min
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        //只对原点周围、高度低于0//原始为z_threshold 的 可能地面点 进行处理
		if (abs(in_cloud_ptr->points[i].x) < size_W && 
            abs(in_cloud_ptr->points[i].y) < size_L && 
            in_cloud_ptr->points[i].z < 0 )
        {
            int cell_i = int(in_cloud_ptr->points[i].y/size_cell) + int(size_L/size_cell); //计算点在栅格xy平面内的具体位置
            int cell_j = int(in_cloud_ptr->points[i].x/size_cell) + int(size_W/size_cell);

            cell_isCover[cell_i][cell_j] = 1; //该位置有 可能地面点
            
            //更新该xy坐标位置的最大z
            if (cell_max_Z[cell_i][cell_j] < in_cloud_ptr->points[i].z)  
                cell_max_Z[cell_i][cell_j] = in_cloud_ptr->points[i].z; 
            
            //更新该xy坐标位置的最小z
            if ( cell_min_Z[cell_i][cell_j] > in_cloud_ptr->points[i].z)  
                cell_min_Z[cell_i][cell_j] = in_cloud_ptr->points[i].z; 
            	
        }
    }

    //核心程序，根据该xy处z的跳变是否太过平缓，判断是否为地面（地面非常平缓）
    for (unsigned int i = 0; i < int(2*size_L/size_cell); i++)
    { 
        for (unsigned int j = 0; j < int(2*size_W/size_cell); j++)
        {
            if (cell_isCover[i][j] == 1 && (cell_max_Z[i][j]-cell_min_Z[i][j]) > ground_remove_threshold_) 
            {
                cell_not_ground[i][j] = 1;
            }
        }
    }

    int max_cell_i = int(2*size_L/size_cell);
    int max_cell_j = int(2*size_W/size_cell);
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        //1.滤除过远点（总栅格尺寸以外）
        int cell_i = int(in_cloud_ptr->points[i].y/size_cell) + int(size_L/size_cell);
        int cell_j = int(in_cloud_ptr->points[i].x/size_cell) + int(size_W/size_cell);

        if (0 <= cell_i && cell_i < max_cell_i && //是下方的if判断的更新，因为下方那个if逻辑三番两次产生漏网之鱼
            0 <= cell_j && cell_j < max_cell_j )
        // if (abs(in_cloud_ptr->points[i].x) < size_W && abs(in_cloud_ptr->points[i].y) < size_L)
        {
            //2.滤除过高点
            if (in_cloud_ptr->points[i].z < z_threshold_)
            {  
                //3.滤除车身点（-0.725是middle到车中心的距离）
                if ((abs(in_cloud_ptr->points[i].x-0.725) > x_threshold_) || (abs(in_cloud_ptr->points[i].y) > y_threshold_))
                {
                    //4.滤除过近点，过近点会影响LOAM定位
                    double dist_point = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2) + pow(in_cloud_ptr->points[i].z, 2));
                    if (dist_point > min_dist_ )  
                    {
                        //5A.滤除地面点
                        // ljx0415：不要轻易删掉第二个条件，因为他滤除了一些容易被误以为是地面点的非地面点，例如：距离地面较高处，
                        // 有一只水平舒展的大胳膊，那么在某些情况下这个大胳膊所处XY栅格的地面点是无法扫描到的（例如人站在雷达附近），
                        // 所以实际该栅格的z_min是大胳膊的下部，z_max是大胳膊的上部，其高度差肯定小于阈值（0.3），就被当成地面点了。
                        if (cell_not_ground[cell_i][cell_j] == 1 || in_cloud_ptr->points[i].z > (ground_z_ + ground_beyond_threshold_))
                        // if (cell_not_ground[cell_i][cell_j] == 1) 
                        {
                            out_without_ground_cloud_ptr->points.push_back(in_cloud_ptr->points[i]); // 保留无地面点，ljx0411
                        }
                        //5B.不滤除地面点
                        else
                        {
                            out_ground_cloud_ptr->points.push_back(in_cloud_ptr->points[i]); // 保留地面点，ljx0411
                        }
                    }
                }
            }
        }
    }

    //6.滤除离散点
    inilerFilterWithoutGround.setInputCloud(out_without_ground_cloud_ptr);
    inilerFilterWithoutGround.filter(*out_without_ground_cloud_ptr);  
    inilerFilterGround.setInputCloud(out_ground_cloud_ptr);
    inilerFilterGround.filter(*out_ground_cloud_ptr);  

    *out_all_cloud_ptr = *out_without_ground_cloud_ptr + *out_ground_cloud_ptr;

    out_without_ground_cloud_ptr->header = in_cloud_ptr->header;
    out_ground_cloud_ptr->header = in_cloud_ptr->header;
    out_all_cloud_ptr->header = in_cloud_ptr->header;

    ROS_INFO("preprocess_core.cpp: ClipCloud has Finished! ");
}

void Preprocessor::ReadStaticYaml(string dir)
{
    m_read_dynamic_yaml_ = new ReadYaml(dir);

    //去过近点
    min_dist_ = m_read_dynamic_yaml_->ReturnMinDist();
    
    //去地面
    leaf_size_ = m_read_dynamic_yaml_->ReturnLeftSize();
    cell_size_ = m_read_dynamic_yaml_->ReturnCellSize();
    z_threshold_ = m_read_dynamic_yaml_->Returnzthreshold();
    y_threshold_ = m_read_dynamic_yaml_->ReturnYThreshold();
    x_threshold_ = m_read_dynamic_yaml_->ReturnXThreshold();
    ground_remove_threshold_ = m_read_dynamic_yaml_->ReturnGroundRemoveThreshold();
    ground_z_ = m_read_dynamic_yaml_->ReturnGroundZ();
    ground_beyond_threshold_ = m_read_dynamic_yaml_->ReturnGroundBeyondThreshold();
    //收发topics
    middle_lidar_sub_topic_ = m_read_dynamic_yaml_->ReturnMiddleLidarSubTopic();
    front_lidar_sub_topic_ = m_read_dynamic_yaml_->ReturnFrontLidarSubTopic();
    left_lidar_sub_topic_ = m_read_dynamic_yaml_->ReturnLeftLidarSubTopic();
    right_lidar_sub_topic_ = m_read_dynamic_yaml_->ReturnRightLidarSubTopic();

    // T_middle2veh = m_read_dynamic_yaml_->ReturnMiddle2Veh();
    T_front2middle_ = m_read_dynamic_yaml_->ReturnFront2Middle();
    T_left2middle_ = m_read_dynamic_yaml_->ReturnLeft2Middle();
    T_right2middle_ = m_read_dynamic_yaml_->ReturnRight2Middle();

    delete m_read_dynamic_yaml_;
}