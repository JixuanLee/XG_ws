#include "lattice_planner/lattice_planner.h"

void Lattice::PubPdtPath(void)
{
    path_pdt_msg.header.stamp = ros::Time::now();
    PathPdtPub.publish(path_pdt_msg);
}

void Lattice::PubPdtPath_Veh(void)
{
    path_pdt_veh_msg.header.stamp = ros::Time(0);
    PdtPathVehPub.publish(path_pdt_veh_msg);
}

void Lattice::PubPathLocal_Veh(void)
{
    path_local_veh_msg.header.stamp = ros::Time(0);
    PathLocalVehPub.publish(path_local_veh_msg);
}

void Lattice::PubOffsetRefPath_Veh(void)
{
    offset_ref_path_veh_msg.header.stamp = ros::Time(0);
    OffsetRefPathVehPub.publish(offset_ref_path_veh_msg);
}

void Lattice::PubLocalPathCluster(void)
{
    LocalPathCluster_msg.header.stamp = ros::Time::now();
    LocalPathClusterPub.publish(LocalPathCluster_msg);
}
