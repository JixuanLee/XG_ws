#include "lattice_planner/lattice_planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_planning");
    ros::NodeHandle nh("~");
    Lattice lattice(nh);
    ros::Rate loop_rate(lattice.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        // 大致流程如下：
        // 1. 将全局路径在局部地图中截断
        // 2. 判断当前是否需要更新路径
        // 3. 需要更新的话
        //   3.1 先根据截断出的路径进行平移，得到路径簇
        //   3.2 然后依次按照固定顺序检测路径簇（依次：防止路径抖动）
        //   3.3 检测过程：
        //       3.3.1 对路径簇的每一条路径，使用纯跟踪算法预测出要走的路径，这个路径的长度是由迭代次数Pdt_len限制的
        //       3.3.2 然后根据预测出的路径进行障碍物检测，检测障碍物的路径长度大致是[0, obs_path_len_min]，
        //             如果当前路径无碰撞，则认为是规划出的新路径； 如果碰撞，则返回3.3.1
        //       3.3.3 如果上述由迭代次数Pdt_len确定的预测路径都碰撞，则减少Pdt_len（即降低预测路径的长度），返回3.3.1


        if (lattice.LatticeFlag())//判断是否收到需要的数据
        {
            lattice.GetLocalPath();//截取全局路径中车辆周围的路径
            if (lattice.WhetherReplan())//检测是否需要重新规划
            {
                lattice.LocalPathOffset();//偏移路径
                lattice.PubLocalPathCluster(); // 所有的偏移路线
                lattice.GenTrajectory();//计算局部路径
            }
                        
            lattice.GenPathMsg();
            lattice.PubPdtPath();           // 最终选择的可以避障的path, in map frame
            lattice.PubPdtPath_Veh();       // 最终选择的可以避障的path, in veh frame
            lattice.PubPathLocal_Veh();     // 只是将全局路径截断后，转换成veh坐标系发布
            lattice.PubOffsetRefPath_Veh(); 
        }
        
        // std::cout << std::endl;
        loop_rate.sleep();        
    }
    
    return 0;
}