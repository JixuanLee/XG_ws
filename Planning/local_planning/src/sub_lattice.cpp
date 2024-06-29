#include "lattice_planner/lattice_planner.h"

Lattice::Lattice(ros::NodeHandle &nh) : listener_(buffer_)
{
    RATE = 10;

    // subscribe
    std::string global_planning_path_topic = nh.param<std::string>("global_planning_path_topic", "/global_planning/sPath");
    std::string odom_topic                 = nh.param<std::string>("odom_topic",                 "/odom_by_imu_gnss");
    std::string local_map_topic            = nh.param<std::string>("local_map_topic",            "/grid_cost_map/local_occupancy_grid_map");       
    PathSub     = nh.subscribe(global_planning_path_topic, 10, &Lattice::PathCallback, this);
    OdomSub     = nh.subscribe(odom_topic, 10, &Lattice::OdomCallback, this);
    LocalMapSub = nh.subscribe(local_map_topic, 10, &Lattice::LocalMapCallback, this);

    // 路径
    PathPdtPub = nh.advertise<nav_msgs::Path>("pdt_path_in_map", 10);
    path_pdt_msg.header.frame_id = "map";

    PdtPathVehPub = nh.advertise<nav_msgs::Path>("pdt_path_in_veh", 10);
    path_pdt_veh_msg.header.frame_id = "veh";

    PathLocalVehPub = nh.advertise<nav_msgs::Path>("local_path_in_veh", 10);
    path_local_veh_msg.header.frame_id = "veh";

    OffsetRefPathVehPub = nh.advertise<nav_msgs::Path>("offset_path_in_veh", 10);
    offset_ref_path_veh_msg.header.frame_id = "veh";

    LocalPathClusterPub = nh.advertise<geometry_msgs::PoseArray>("offset_path_clusters", 10);
    LocalPathCluster_msg.header.frame_id = "map";

    // 路径
    Path_Size_min = 3;
    Path_Length_min = 0.2;

    // 旗标
    map_flag = false;
    path_flag = false;
    odom_flag = false;
    replan_by_new_path = false;

    // 碰撞检测
    nh.param<float>("LocalMapHalfWidth", LocalMapHalfWidth,15.0f);
    nh.param<float>("Veh_L", Veh_L, 3.2f);
    nh.param<float>("OBS_DIS", OBS_DIS, 1.1f);
    nh.param<float>("obs_path_len_min", obs_path_len_min, 10.0f);
    nh.param<float>("obs_path_len_max", obs_path_len_max, 15.0f);

    // 路径偏移
    nh.param<int>("OFFSET_NUM", OFFSET_NUM, 15);
    nh.param<float>("OFFSET_DIS", OFFSET_DIS, 0.4f);
    LocalPathCluster.resize(2 * OFFSET_NUM + 1);
    path_local_index = OFFSET_NUM;
    
    // 预瞄
    nh.param<float>("PRE_DIS", PRE_DIS, 2.8f);
    nh.param<float>("u_weight", u_weight, 0.3f);
    nh.param<int>("Pdt_len_max", Pdt_len_max, 300);
    nh.param<int>("Pdt_len_min", Pdt_len_min, 80);
    nh.param<int>("Pdt_len_step", Pdt_len_step, 10);
    nh.param<float>("ANGLE_MIN", ANGLE_MIN, 1e-4f);
    nh.param<float>("U_MIN", U_MIN, 0.7f);
    nh.param<float>("W_MAX", W_MAX, 1.0f);
    
    // 路径拟合
    Poly_Rank = 3;
}

void Lattice::LocalMapCallback(const nav_msgs::OccupancyGrid &msg)
{
    map_o_x = msg.info.origin.position.x;
    map_o_y = msg.info.origin.position.y;
    map_rows = msg.info.height;
    map_cols = msg.info.width;
    map_res = msg.info.resolution;

    image_map.create(map_rows, map_cols, CV_8UC1);
    for (int i = 0; i < map_rows; i++)
    {
        for (int j = 0; j < map_cols; j++)
        {
            if (msg.data[i * map_cols + j] >= 0 && msg.data[i * map_cols + j] <= 70)
            {
                image_map.at<uchar>(map_rows - 1 - i, j) = 255;//255 白色 可通行
            }
            else
            {
                image_map.at<uchar>(map_rows - 1 - i, j) = 0; // 0 黑色 不可通行
            }
        }
    }

    image_dis.create(map_rows, map_cols, CV_32FC1);
    cv::distanceTransform(image_map, image_dis, cv::DIST_L2, 5, CV_32FC1);

    //--------------------------------------------------------------------------------------------
    // 打开下两段注释，可以查看二维栅格地图与二值化地图
    //--------------------------------------------------------------------------------------------

    // cv::Mat distShow;
    // distShow = cv::Mat::zeros(image_dis.size(), CV_8UC1); // 定义细化后的字符轮廓
    // for (int i = 0; i < image_dis.rows; i++)
    // {
    //     for (int j = 0; j < image_dis.cols; j++)
    //     {
    //         distShow.at<uchar>(i, j) = image_dis.at<float>(i, j);
    //     }
    // }
    // cv::normalize(distShow, distShow, 0, 255, cv::NORM_MINMAX); // 为了显示清晰，做了0~255归一化
    // static int cnt = 0;
    // if (++cnt % 20 == 0)
    // {
    //     cv::Mat file; 
    //     cv::hconcat(image_map, distShow, file);

    //     std::string name = "/home/brucesun/XG_ws/src/Planning/local_planning/img/" + std::to_string(cnt / 20) + ".png";
    //     cv::imwrite(name, file);
    //     std::cout << "saved \"" << name << "\" to files" << std::endl;
    // }
    

    // cv::imshow("img", image_map);
    // cv::imshow("dis", distShow);
    // cv::waitKey(0.1);

    map_flag = true;
}

void Lattice::PathCallback(const nav_msgs::Path &msg)
{
    if (msg.poses.size() < 3)
    {
        path_flag = false;
        return;
    }

    path_ref.resize(msg.poses.size());
    for (size_t i = 0; i < path_ref.size(); i++)
    {
        path_ref[i].x = msg.poses[i].pose.position.x;
        path_ref[i].y = msg.poses[i].pose.position.y;
    }

    replan_by_new_path = true;
    path_flag = true;
}

void Lattice::OdomCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::TransformStamped tfs;
    try
    {
        tfs = buffer_.lookupTransform("map", "veh", ros::Time(0));
        state_now.x = tfs.transform.translation.x;
        state_now.y = tfs.transform.translation.y;
        state_now.yaw = tf::getYaw(tfs.transform.rotation);
        state_now.u = msg.twist.twist.linear.x;
    }
    catch (const tf2::TransformException &e)
    {
        ROS_ERROR_THROTTLE(1, "%s", e.what());
        odom_flag = false;
        return;
    }

    odom_flag = true;
}
