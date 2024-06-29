#include "lattice_planner/lattice_planner.h"

bool Lattice::LatticeFlag(void)
{
    ROS_INFO_THROTTLE(1, "map_flag = %d, path_flag = %d, odom_flag = %d", map_flag, path_flag, odom_flag);
    return map_flag && path_flag && odom_flag;
}

void Lattice::GetLocalPath(void)
{
    // 获得当前车辆在全局路径中的最近点，然后根据这个最近点开始截断全局路径
    near_index_ref_path = FindNearPoint(path_ref, state_now);
    path_local.clear();
    path_local_veh.clear();

    PathNode node_tmp;
    for (size_t i = near_index_ref_path; i < path_ref.size(); i++)
    {
        // 大地坐标系转换为车辆坐标系
        node_tmp.x = (path_ref[i].x - state_now.x) * cos(state_now.yaw) +
                     (path_ref[i].y - state_now.y) * sin(state_now.yaw);

        node_tmp.y = -(path_ref[i].x - state_now.x) * sin(state_now.yaw) +
                     (path_ref[i].y - state_now.y) * cos(state_now.yaw);

        // 采用tf--0521
        // geometry_msgs::PointStamped path_point_in_map;  
        // geometry_msgs::PointStamped path_point_in_veh;
        // path_point_in_map.header.stamp = ros::Time();
        // path_point_in_map.header.frame_id = "map";
        // path_point_in_map.point.x = path_ref[i].x;
        // path_point_in_map.point.y = path_ref[i].y;
        // path_point_in_map.point.z = 0;
        // // ROS_INFO_THROTTLE(0.25, "[3] Aim's  y---x-inMap: %.3f , %.3f", aim_point_in_map.point.y, aim_point_in_map.point.x);

        // try{
        //     node_tmp = buffer_.transform(path_point_in_map,"veh");
        // }
        // catch(const std::exception& e){
        //     ROS_INFO("TF ERROR:%s",e.what());
        // }
        // 采用tf结束--0521

        // 截取地图内的点
        if (node_tmp.x > -map_res * map_cols / 2 && node_tmp.x < map_res * map_cols / 2 &&
            node_tmp.y > -map_rows * map_res / 2 && node_tmp.y < map_rows * map_res / 2)
        {
            path_local.push_back(path_ref[i]);
            path_local_veh.push_back(node_tmp);
        }
        else
            break;
    }

    // 计算路径长度 角度 曲率信息
    path_local[0].phi = atan2(path_local[1].y - path_local[0].y, path_local[1].x - path_local[0].x);
    path_local[0].s = 0;
    path_local_veh[0].phi = atan2(path_local_veh[1].y - path_local_veh[0].y, path_local_veh[1].x - path_local_veh[0].x);
    path_local_veh[0].s = 0;
    for (size_t i = 1; i < path_local.size(); i++)
    {
        path_local[i].s = path_local[i - 1].s + sqrt(pow((path_local[i].y - path_local[i - 1].y), 2) + pow((path_local[i].x - path_local[i - 1].x), 2));
        path_local_veh[i].s = path_local_veh[i - 1].s + sqrt(pow((path_local_veh[i].y - path_local_veh[i - 1].y), 2) + pow((path_local_veh[i].x - path_local_veh[i - 1].x), 2));

        if (i == path_local.size() - 1)
        {
            path_local[i].phi = atan2(path_local[i].y - path_local[i - 1].y, path_local[i].x - path_local[i - 1].x);
            path_local_veh[i].phi = atan2(path_local_veh[i].y - path_local_veh[i - 1].y, path_local_veh[i].x - path_local_veh[i - 1].x);
        }
        else
        {
            path_local[i].phi = atan2(path_local[i + 1].y - path_local[i - 1].y, path_local[i + 1].x - path_local[i - 1].x);
            path_local_veh[i].phi = atan2(path_local_veh[i + 1].y - path_local_veh[i - 1].y, path_local_veh[i + 1].x - path_local_veh[i - 1].x);
        }

        float phi_delta = path_local[i].phi - path_local[i - 1].phi;

        if (phi_delta > M_PI)
        {
            phi_delta = phi_delta - 2 * M_PI;
            phi_delta = -phi_delta;
        }

        if (phi_delta <= -M_PI)
        {
            phi_delta = phi_delta + 2 * M_PI;
            phi_delta = -phi_delta;
        }

        path_local[i].c = phi_delta / (path_local[i].s - path_local[i - 1].s);
    }
    path_local[0].c = path_local[1].c;

    if (path_local.back().s < Path_Length_min)
    {
        path_local.clear();
        path_local_veh.clear();
        return;
    }

    std::cout << "path_local length = " << path_local.size() << " points, " << path_local.back().s << " m" << std::endl;
}

int Lattice::FindNearPoint(std::vector<PathNode> &path, VehState &veh) // 寻找车辆最近的路点
{
    int near_index = 0;
    int dis_min = 10000;

    for (size_t i = 0; i < path.size(); i++)
    {
        int dis_tmp = sqrt(pow(path[i].x - veh.x, 2) + pow(path[i].y - veh.y, 2));
        if (dis_tmp < dis_min)
        {
            dis_min = dis_tmp;
            near_index = i;
        }
    }
    return near_index;
}

void Lattice::PathWorld2Veh(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v)
{
    if (path_w.size() < 2)
        return;
    path_v.resize(path_w.size());

    for (size_t i = 0; i < path_v.size(); i++)
    {
        path_v[i].x = (path_w[i].x - state_now.x) * cos(state_now.yaw) +
                      (path_w[i].y - state_now.y) * sin(state_now.yaw); //+ (map_o_x + map_rows/2 * map_res);

        path_v[i].y = -(path_w[i].x - state_now.x) * sin(state_now.yaw) +
                      (path_w[i].y - state_now.y) * cos(state_now.yaw); //+ (map_o_y + map_cols/2 * map_res);
    }

    for (size_t i = 0; i < path_v.size(); i++)
    {
        if (i == 0)
        {
            path_v[i].phi = atan2(path_v[i + 1].y - path_v[i].y, path_v[i + 1].x - path_v[i].x);
            path_v[i].s = 0;
        }
        else
        {
            path_v[i].s = path_v[i - 1].s + sqrt(pow((path_v[i].x - path_v[i - 1].x), 2) + pow((path_v[i].y - path_v[i - 1].y), 2));
            if (i == path_v.size() - 1)
                path_v[i].phi = atan2(path_v[i].y - path_v[i - 1].y, path_v[i].x - path_v[i - 1].x);
            else
                path_v[i].phi = atan2(path_v[i + 1].y - path_v[i - 1].y, path_v[i + 1].x - path_v[i - 1].x);
        }
    }
}

void Lattice::PathVeh2World(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v)
{
    if (path_v.size() < 2)
        return;
    path_w.resize(path_v.size());

    for (size_t i = 0; i < path_v.size(); i++)
    {
        path_w[i].x = path_v[i].x * cos(-state_now.yaw) + path_v[i].y * sin(-state_now.yaw) + state_now.x;
        path_w[i].y = -path_v[i].x * sin(-state_now.yaw) + path_v[i].y * cos(-state_now.yaw) + state_now.y;
    }

    for (size_t i = 0; i < path_w.size(); i++)
    {
        if (i == 0)
        {
            path_w[i].phi = atan2(path_w[i + 1].y - path_w[i].y, path_w[i + 1].x - path_w[i].x);
        }
        else
        {
            if (i == path_w.size() - 1)
                path_w[i].phi = atan2(path_w[i].y - path_w[i - 1].y, path_w[i].x - path_w[i - 1].x);
            else
                path_w[i].phi = atan2(path_w[i + 1].y - path_w[i - 1].y, path_w[i + 1].x - path_w[i - 1].x);
        }
    }
}

bool Lattice::ObsCheck(std::vector<PathNode> &path, int near_index) // 对预测路径进行碰撞检测
{
    if (path.size() < 2)
        return true;
    
    float obs_path_len = obs_path_len_min + state_now.u;

    if (obs_path_len > obs_path_len_max)
        obs_path_len = obs_path_len_max;

    int index_row = 0;
    int index_col = 0;
    for (size_t i = near_index; i < path.size(); i++)
    {
        if (path[i].s - path[near_index].s < obs_path_len)
        {
            // 三碰撞圆检测
            index_row = map_rows - 1 - floor((path[i].y - Veh_L * sin(path[i].phi) / 4 + LocalMapHalfWidth) / map_res);
            index_col = floor((path[i].x - Veh_L * cos(path[i].phi) / 4 + LocalMapHalfWidth) / map_res);
            if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
            {
                if (image_dis.at<float>(index_row, index_col) * map_res < OBS_DIS)
                {
                    return true;
                }
            }

            index_row = map_rows - 1 - floor((path[i].y + Veh_L * sin(path[i].phi) / 4 + LocalMapHalfWidth) / map_res);
            index_col = floor((path[i].x + Veh_L * cos(path[i].phi) / 4 + LocalMapHalfWidth) / map_res);

            if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
            {
                if (image_dis.at<float>(index_row, index_col) * map_res < OBS_DIS)
                {
                    return true;
                }
            }

            index_row = map_rows - 1 - floor((path[i].y + LocalMapHalfWidth) / map_res);
            index_col = floor((path[i].x + LocalMapHalfWidth) / map_res);

            if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
            {
                if (image_dis.at<float>(index_row, index_col) * map_res < OBS_DIS)
                {
                    return true;
                }
            }
        }
        else
        {
            break;
        }
    }

    return false;
}

bool Lattice::WhetherReplan(void)
{
    if (path_local.empty())
        return false;

    // 若无预测轨迹，重规划
    if (path_pdt.empty())
    {
        std::cout << "replan: path_pdt is empty" << std::endl;
        return true;
    }
    // 若有新的全局路径，立刻重规划
    else if (replan_by_new_path)
    {
        std::cout << "replan: new global path" << std::endl;
        replan_by_new_path = false;
        return true;
    }
    // 否则根据当前路径的状态决定是否需要更新
    else
    {
        // 车辆当前在参考路径上的最近点位置为near_index_ref_path，已经在截断全局路径的时候获得
        // 若车辆当前已经到达了全局路径的末端附近(最近点是全局路径上的最后2个点)，认为全局规划任务完成，后续停止局部规划
        if (near_index_ref_path >= path_ref.size() - 2)
        {
            std::cout << "replan: local planning finished! dont need to replan" << std::endl;
            path_flag = false;
            return false;
        }

        // 若到达预测轨迹距离阈值，重规划
        int near_index = FindNearPoint(path_pdt, state_now);
        float s_threshold = std::max(path_pdt.back().s / 2, 6.0f); // 重规划距离阈值
        if (path_pdt[near_index].s > s_threshold)
        {
            std::cout << "replan: path_pdt should be updated" << std::endl;
            return true;
        }

        // 若预测轨迹有障碍，重规划
        PathWorld2Veh(path_pdt, path_pdt_veh);
        if (ObsCheck(path_pdt_veh, near_index))
        {
            std::cout << "replan: path_pdt is blocked" << std::endl;
            return true;
        }
    }

    std::cout << "Do not need to replan" << std::endl;
    return false;
}

void Lattice::LocalPathOffset(void)
{
    std::vector<bool> trim_flag(2 * OFFSET_NUM + 1, false); // 修剪旗标
    for (int i = 0; i < LocalPathCluster.size(); i++)
    {
        LocalPathCluster[i].clear();
        float offset = (OFFSET_NUM - i) * OFFSET_DIS; // 路径偏移距离

        PathNode node_tmp;
        for (size_t j = 0; j < path_local.size(); j++)
        {
            // 若曲率为0或曲率半径大于偏移距离，则偏移；否则跳过该路点，需要修剪
            if (path_local[j].c == 0 || fabs(1 / path_local[j].c) > fabs(offset))
            {
                node_tmp.x = path_local[j].x - offset * sin(path_local[j].phi);
                node_tmp.y = path_local[j].y + offset * cos(path_local[j].phi);
                // node_tmp.phi = path_local[j].phi;

                LocalPathCluster[i].push_back(node_tmp);
            }
            else
                trim_flag[i] = true;
        }
    }

    for (size_t i = 0; i < LocalPathCluster.size(); i++)
    {
        if (trim_flag[i])
        {
            if (LocalPathCluster[i].size() > 2)
                Trim(LocalPathCluster[i]);

            if (LocalPathCluster[i].size() > Poly_Rank)
                Polyfit(LocalPathCluster[i]);
        }
        else
        {
            for (size_t j = 0; j < LocalPathCluster[i].size(); j++)
            {
                if (j == 0)
                {
                    LocalPathCluster[i][j].s = 0;
                    if (LocalPathCluster[i].size() > 1)
                    {
                        LocalPathCluster[i][j].phi = atan2(LocalPathCluster[i][j + 1].y - LocalPathCluster[i][j].y,
                                                           LocalPathCluster[i][j + 1].x - LocalPathCluster[i][j].x);
                    }
                }
                else
                {
                    LocalPathCluster[i][j].s = LocalPathCluster[i][j - 1].s + sqrt(pow(LocalPathCluster[i][j].x - LocalPathCluster[i][j - 1].x, 2) +
                                                                                   pow(LocalPathCluster[i][j].y - LocalPathCluster[i][j - 1].y, 2));

                    if (LocalPathCluster[i].size() > 1)
                    {
                        if (j == LocalPathCluster[i].size() - 1)
                        {
                            LocalPathCluster[i][j].phi = atan2(LocalPathCluster[i][j].y - LocalPathCluster[i][j - 1].y,
                                                               LocalPathCluster[i][j].x - LocalPathCluster[i][j - 1].x);
                        }
                        else
                        {
                            LocalPathCluster[i][j].phi = atan2(LocalPathCluster[i][j + 1].y - LocalPathCluster[i][j - 1].y,
                                                               LocalPathCluster[i][j + 1].x - LocalPathCluster[i][j - 1].x);
                        }
                    }
                }
            }
        }
    }

    // msg部分
    LocalPathCluster_msg.poses.clear();
    for (size_t i = 0; i < LocalPathCluster.size(); i++)
    {
        for (size_t j = 0; j < LocalPathCluster[i].size(); j++)
        {
            geometry_msgs::Pose pose_msg;
            pose_msg.position.x = LocalPathCluster[i][j].x;
            pose_msg.position.y = LocalPathCluster[i][j].y;
            pose_msg.orientation = tf::createQuaternionMsgFromYaw(LocalPathCluster[i][j].phi);

            LocalPathCluster_msg.poses.push_back(pose_msg);
        }
    }

    std::cout << "Path offset finished" << std::endl;

    // for (size_t i = 0; i < LocalPathCluster.size(); i++)
    //     std::cout << "LocalPathCluster[" << i << "].s = " << LocalPathCluster[i][LocalPathCluster[i].size()-1].s << "m" << std::endl;
}

void Lattice::Trim(std::vector<PathNode> &path)
{
    std::vector<PathNode> path_tmp;
    int k = -1; // 距离最近点下标

    PathNode node_tmp;
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        node_tmp.x = path[i].x;
        node_tmp.y = path[i].y;
        path_tmp.push_back(node_tmp);

        int dis_min = path[i + 1].s - path[i].s;
        for (size_t j = i + 2; j < path.size() - 1; j++)
        {
            int dis_tmp = sqrt(pow((path[j].x - path[i].x), 2) + pow((path[j].y - path[i].y), 2));
            if (dis_tmp <= dis_min)
            {
                k = j;
                dis_min = dis_tmp;
            }
        }

        if (k != -1)
        {
            i = k - 1;
            k = -1;
        }
    }

    // 去掉尾部因偏移修剪偏差过大的路点
    if (path_tmp.size() > 2)
    {
        if (sqrt(pow((path_tmp[path_tmp.size() - 1].y - path_tmp[path_tmp.size() - 2].y), 2) + pow((path_tmp[path_tmp.size() - 1].x - path_tmp[path_tmp.size() - 2].x), 2)) >
            sqrt(pow((path_tmp[path_tmp.size() - 2].y - path_tmp[path_tmp.size() - 3].y), 2) + pow((path_tmp[path_tmp.size() - 2].x - path_tmp[path_tmp.size() - 3].x), 2)))
            path_tmp.pop_back();
    }

    path.resize(path_tmp.size());
    for (size_t i = 0; i < path.size(); i++)
    {
        path[i].x = path_tmp[i].x;
        path[i].y = path_tmp[i].y;

        if (i == 0)
        {
            path[i].s = 0;

            if (path.size() > 1)
                path[i].phi = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
        }
        else
        {
            path[i].s = path[i - 1].s + sqrt(pow((path[i].x - path[i - 1].x), 2) + pow((path[i].y - path[i - 1].y), 2));

            if (path.size() > 1)
            {
                if (i == path.size() - 1)
                    path[i].phi = atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
                else
                    path[i].phi = atan2(path[i + 1].y - path[i - 1].y, path[i + 1].x - path[i - 1].x);
            }
        }
    }
}

void Lattice::Polyfit(std::vector<PathNode> &path)
{
    int s_col = Poly_Rank + 1;
    A.resize(s_col);
    B.resize(s_col);

    int s_row = path.size();
    S.resize(s_row, s_col);
    P.resize(s_row);

    for (size_t i = 0; i < S.rows(); i++)
    {
        for (size_t j = 0; j < S.cols(); j++)
            S(i, j) = pow(path[i].s, j);
    }

    for (size_t i = 0; i < P.size(); i++)
    {
        P[i] = path[i].x;
    }
    A = S.colPivHouseholderQr().solve(P);

    for (size_t i = 0; i < P.size(); i++)
    {
        P[i] = path[i].y;
    }
    B = S.colPivHouseholderQr().solve(P);

    // 拟合步长

    float s_start = path.front().s;
    float s_end = path.back().s;
    float poly_step = (s_end - s_start) / (path.size() - 1);
    path.clear();
    PathNode node_tmp;
    for (float i = s_start; i < s_end - 0.01; i = i + poly_step)
    {
        node_tmp.x = 0;
        node_tmp.y = 0;

        for (size_t j = 0; j < A.size(); j++)
        {
            node_tmp.x += A[j] * pow(i, j);
            node_tmp.y += B[j] * pow(i, j);
        }

        path.push_back(node_tmp);
    }

    for (size_t i = 0; i < path.size(); i++)
    {
        if (i == 0)
        {
            path[i].s = 0;
            if (path.size() > 1)
                path[i].phi = atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
        }
        else
        {
            path[i].s = path[i - 1].s + sqrt(pow((path[i].x - path[i - 1].x), 2) + pow((path[i].y - path[i - 1].y), 2));
            if (path.size() > 1)
            {
                if (i == path.size() - 1)
                    path[i].phi = atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
                else
                    path[i].phi = atan2(path[i + 1].y - path[i - 1].y, path[i + 1].x - path[i - 1].x);
            }
        }
    }
}

void Lattice::GenTrajectory(void) // 对截取的全局路径以及偏移的路径进行跟踪预测，若成功，则输出局部路径
{
    for (int len_num = Pdt_len_max; len_num >= Pdt_len_min; len_num = len_num - Pdt_len_step)
    {
        for (int i = 0; i < OFFSET_NUM; i++)
        {
            Track(LocalPathCluster[OFFSET_NUM + i], len_num);
            PathWorld2Veh(path_pdt, path_pdt_veh);
            if (!ObsCheck(path_pdt_veh, 0))
            {
                path_local_index = OFFSET_NUM + i;
                return;
            }
            // ROS_WARN("path %d: obstacle check failed..", OFFSET_NUM + i);

            if (i == 0)
                continue;

            Track(LocalPathCluster[OFFSET_NUM - i], len_num);
            PathWorld2Veh(path_pdt, path_pdt_veh);
            if (!ObsCheck(path_pdt_veh, 0))
            {
                path_local_index = OFFSET_NUM - i;
                return;
            }
            // ROS_WARN("path %d: obstacle check failed..", OFFSET_NUM - i);
        }
    }

    path_pdt.clear();
    path_pdt_veh.clear();
    std::cout << "Replan Failed" << std::endl;
}

void Lattice::Track(std::vector<PathNode> &path, int len) // 使用二自由度车辆模型对路径进行跟踪预测
{
    if (path.size() < Path_Size_min)
        return;

    path_pdt.clear();
    float pre_dis = PRE_DIS + u_weight * state_now.u;
    VehState state_tmp = state_now;

    int pre_aim; // 预瞄点下标
    for (size_t i = 1; i < len; i++)
    {
        int track_beg = FindNearPoint(path, state_tmp);
        int dis_tmp = sqrt(pow(path[track_beg].x - state_tmp.x, 2) + pow(path[track_beg].y - state_tmp.y, 2));
        if (dis_tmp >= pre_dis)
        {
            pre_aim = track_beg;
        }
        else
        {
            for (size_t j = track_beg; j < path.size(); j++)
            {
                if (path[j].s - path[track_beg].s > pre_dis)
                {
                    pre_aim = j;
                    break;
                }

                if (j == path.size() - 1)
                    pre_aim = path.size() - 1;
            }
        }

        if (pre_aim == track_beg && pre_aim == path.size() - 1)
        {
            break;
        }

        dis_tmp = sqrt(pow(path[pre_aim].x - state_tmp.x, 2) + pow(path[pre_aim].y - state_tmp.y, 2));
        float pre_angle = atan2(path[pre_aim].y - state_tmp.y, path[pre_aim].x - state_tmp.x) - state_tmp.yaw;

        float cmd_u = state_tmp.u;
        if (cmd_u < U_MIN)
            cmd_u = U_MIN;

        float cmd_w = 0;
        if (fabs(pre_angle) > ANGLE_MIN)
        {
            float pre_r = dis_tmp / 2 / sin(pre_angle);
            cmd_w = cmd_u / pre_r;

            if (cmd_w > W_MAX)
                cmd_w = W_MAX;
            if (cmd_w < -W_MAX)
                cmd_w = -W_MAX;
        }

        // 车辆运动学模型
        state_tmp.x = state_tmp.x + cmd_u * cos(state_tmp.yaw) / RATE;
        state_tmp.y = state_tmp.y + cmd_u * sin(state_tmp.yaw) / RATE;
        state_tmp.yaw = state_tmp.yaw + cmd_w / RATE;

        while (state_tmp.yaw > M_PI)
            state_tmp.yaw -= 2 * M_PI;
        while (state_tmp.yaw <= -M_PI)
            state_tmp.yaw += 2 * M_PI;

        PathNode node_tmp;
        node_tmp.x = state_tmp.x;
        node_tmp.y = state_tmp.y;
        node_tmp.phi = state_tmp.yaw;
        path_pdt.push_back(node_tmp);
    }

    for (size_t i = 0; i < path_pdt.size(); i++)
    {
        if (i == 0)
            path_pdt[i].s = 0;
        else
            path_pdt[i].s = path_pdt[i - 1].s + sqrt(pow((path_pdt[i].x - path_pdt[i - 1].x), 2) + pow((path_pdt[i].y - path_pdt[i - 1].y), 2));
    }
}

void Lattice::GenPathMsg(void)
{
    if (path_local.empty())
    {
        path_pdt_msg.poses.clear();
        path_pdt_veh_msg.poses.clear();
        path_local_veh_msg.poses.clear();
        offset_ref_path_veh_msg.poses.clear();

        std::cout << "path_local is too short" << std::endl;
        return;
    }

    path_local_veh_msg.poses.resize(path_local_veh.size());
    for (size_t i = 0; i < path_local_veh_msg.poses.size(); i++)
    {
        path_local_veh_msg.poses[i].pose.position.x = path_local_veh[i].x;
        path_local_veh_msg.poses[i].pose.position.y = path_local_veh[i].y;
        path_local_veh_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_local_veh[i].phi);
    }

    path_pdt_msg.poses.resize(path_pdt.size());
    for (size_t i = 0; i < path_pdt_msg.poses.size(); i++)
    {
        path_pdt_msg.poses[i].pose.position.x = path_pdt[i].x;
        path_pdt_msg.poses[i].pose.position.y = path_pdt[i].y;
        path_pdt_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_pdt[i].phi);
    }

    PathWorld2Veh(path_pdt, path_pdt_veh);
    path_pdt_veh_msg.poses.resize(path_pdt_veh.size());
    for (size_t i = 0; i < path_pdt_veh_msg.poses.size(); i++)
    {
        path_pdt_veh_msg.poses[i].pose.position.x = path_pdt_veh[i].x + (map_o_x + map_rows / 2 * map_res);
        path_pdt_veh_msg.poses[i].pose.position.y = path_pdt_veh[i].y + (map_o_y + map_rows / 2 * map_res);
        path_pdt_veh_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_pdt_veh[i].phi);
    }

    std::vector<PathNode> path_tmp;
    PathWorld2Veh(LocalPathCluster[path_local_index], path_tmp);
    offset_ref_path_veh_msg.poses.resize(path_tmp.size());
    for (size_t i = 0; i < offset_ref_path_veh_msg.poses.size(); i++)
    {
        offset_ref_path_veh_msg.poses[i].pose.position.x = path_tmp[i].x;
        offset_ref_path_veh_msg.poses[i].pose.position.y = path_tmp[i].y;
        offset_ref_path_veh_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_tmp[i].phi);
    }

    std::cout << "path_pdt.size() = " << path_pdt.size() << std::endl;
}
