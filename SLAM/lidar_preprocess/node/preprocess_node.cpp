#include "lidar_preprocess/preprocess.h"


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "lidar_preprocess");

    Preprocessor app;
    app.Run();

    return 0;
}
