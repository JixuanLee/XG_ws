#include "path_tracking/path_tracking.h"

void PathTracking::PubCmd()
{
    CmdPub.publish(cmd_msg);
}

void PathTracking::PubBrake()
{
    BrakePub.publish(brake_msg); // brake_msg.data = false/ture
}

void PathTracking::PubRear()
{
    RearPub.publish(rear_msg);
}