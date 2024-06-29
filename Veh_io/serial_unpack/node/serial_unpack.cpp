#include "serial_unpack/inclinometer_serial_manager.h"

int main(int argc, char *argv[])
{  
    ros::init(argc, argv, "serial_unpack");
    ros::NodeHandle nh("~");

    bool inclinometer_success = false;
    Inclinometer::InclinometerSerialManager inclinometer_unpack(nh, inclinometer_success);
    if (inclinometer_success == false)
    {
        ROS_ERROR("serial unpack failed ...");
        return -1;
    }
    inclinometer_unpack.run();

    ros::spin();
    return 0;  
}