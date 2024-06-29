#include "lego_loam/utility.h"


class TransformFusion
{
private:
    ros::NodeHandle nh;

    nav_msgs::Odometry laserOdometry2;
    ros::Publisher pubLaserOdometry2;
    nav_msgs::Odometry laserOdometry3;
    ros::Publisher pubLaserOdometry3;

    ros::Subscriber subLaserOdometry;
    ros::Subscriber subOdomAftMapped;

    tf::StampedTransform laserOdometryTrans2;
    tf::TransformBroadcaster tfBroadcaster2;

    geometry_msgs::TransformStamped map_2_odom_trans;
    tf2_ros::StaticTransformBroadcaster tfBroadcasterMap2Odom;


    float transformSum[6];
    float transformIncre[6];
    float transformMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    std_msgs::Header currentHeader;

public:
    TransformFusion()
    {
        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry>("/integrated_to_init", 5);
        pubLaserOdometry3 = nh.advertise<nav_msgs::Odometry>("/integrated_to_init_trans", 5); // ljx: 矫正方向，使得odom的箭头与实际运动方向一致
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_to_init", 5, &TransformFusion::laserOdometryHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 5, &TransformFusion::odomAftMappedHandler, this);


        laserOdometry2.header.frame_id = "/vodom"; // 原为camera_init
        laserOdometry2.child_frame_id = "/vveh";   // 原为camera

        laserOdometry3.header.frame_id = "/vodom"; // 原为camera_init  ljx
        laserOdometry3.child_frame_id = "/vveh";   // 原为camera  ljx

        laserOdometryTrans2.frame_id_ = "/vodom";      // 原为camera_init
        laserOdometryTrans2.child_frame_id_ = "/vveh"; // 原为camera

        map_2_odom_trans.header.frame_id = "/map";
        map_2_odom_trans.child_frame_id = "/odom";
        

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }

        pubMap2OdomTrans();
    }

    void transformAssociateToMap()
    {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
        float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]),
                                   crycrx / cos(transformMapped[0]));

        float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]),
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
    {
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;

        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;

        transformAssociateToMap();

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped[2], -transformMapped[0], -transformMapped[1]);

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry2.pose.pose.orientation.z = geoQuat.x;
        laserOdometry2.pose.pose.orientation.w = geoQuat.w;
        laserOdometry2.pose.pose.position.x = transformMapped[3];
        laserOdometry2.pose.pose.position.y = transformMapped[4];
        laserOdometry2.pose.pose.position.z = transformMapped[5];
        pubLaserOdometry2.publish(laserOdometry2);

        // ljx   纠正odom方向与实际XG车辆运动方向不一致的情况
        tf2::Quaternion rotate90, quatSrc, combinedQuat;
        rotate90.setRPY(-M_PI / 2, -M_PI / 2, 0);
        quatSrc.setX(laserOdometry2.pose.pose.orientation.x);
        quatSrc.setY(laserOdometry2.pose.pose.orientation.y);
        quatSrc.setZ(laserOdometry2.pose.pose.orientation.z);
        quatSrc.setW(laserOdometry2.pose.pose.orientation.w);
        combinedQuat = rotate90 * quatSrc;

        laserOdometry3.header.stamp = laserOdometry->header.stamp;
        laserOdometry3.pose.pose.orientation.x = combinedQuat.x();
        laserOdometry3.pose.pose.orientation.y = combinedQuat.y();
        laserOdometry3.pose.pose.orientation.z = combinedQuat.z();
        laserOdometry3.pose.pose.orientation.w = combinedQuat.w();
        laserOdometry3.pose.pose.position.x = laserOdometry2.pose.pose.position.x;
        laserOdometry3.pose.pose.position.y = laserOdometry2.pose.pose.position.y;
        laserOdometry3.pose.pose.position.z = laserOdometry2.pose.pose.position.z;
        pubLaserOdometry3.publish(laserOdometry3);
        // ljx

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        tfBroadcaster2.sendTransform(laserOdometryTrans2);
    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped)
    {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = -pitch;
        transformAftMapped[1] = -yaw;
        transformAftMapped[2] = roll;

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

        transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
        transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
        transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

        transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
        transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
        transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
    }

    // void gnssHandler(const nav_msgs::Odometry::ConstPtr &gnss_ptr)
    // {
    //     // double yaw_map_to_veh = tf2::getYaw(gnss.pose.pose.orientation);
    //     // double yaw_veh_to_odom = -tf2::getYaw(laserOdometry2.pose.pose.orientation);
    //     // double yaw_map_to_odom = yaw_map_to_veh + yaw_veh_to_odom;
    //     // if (yaw_map_to_odom > M_PI)
    //     //     yaw_map_to_odom -= 2 * M_PI;
    //     // if (yaw_map_to_odom < -M_PI)
    //     //     yaw_map_to_odom += 2 * M_PI;
        
    //     // map_2_odom_Trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_map_to_odom);


    //     tf2::Quaternion map_to_veh;
    //     map_to_veh.setX(gnss.pose.pose.orientation.x);
    //     map_to_veh.setY(gnss.pose.pose.orientation.y);
    //     map_to_veh.setZ(gnss.pose.pose.orientation.z);
    //     map_to_veh.setW(gnss.pose.pose.orientation.w);

    //     tf2::Quaternion veh_to_odom;
    //     veh_to_odom.setX(-laserOdometry2.pose.pose.orientation.x);
    //     veh_to_odom.setY(-laserOdometry2.pose.pose.orientation.y);
    //     veh_to_odom.setZ(-laserOdometry2.pose.pose.orientation.z);
    //     veh_to_odom.setW(laserOdometry2.pose.pose.orientation.w);
    //     // geometry_msgs::TransformStamped tf_veh2odom;
    //     // try
    //     // {
    //     //     tf_veh2odom = buffer_.lookupTransform("veh", "odom", ros::Time(0), ros::Duration(0.1));
    //     //     veh_to_odom.setX(tf_veh2odom.transform.rotation.x);
    //     //     veh_to_odom.setY(tf_veh2odom.transform.rotation.y);
    //     //     veh_to_odom.setZ(tf_veh2odom.transform.rotation.z);
    //     //     veh_to_odom.setW(tf_veh2odom.transform.rotation.w);
    //     // }        
    //     // catch (tf2::TransformException & ex) 
    //     // {
    //     //     ROS_WARN_THROTTLE(1, "%s", ex.what());
    //     // }

    //     tf2::Quaternion map_to_odom = veh_to_odom * map_to_veh;
    //     map_2_odom_Trans.transform.rotation.x = map_to_odom.x();
    //     map_2_odom_Trans.transform.rotation.y = map_to_odom.y();
    //     map_2_odom_Trans.transform.rotation.z = map_to_odom.z();
    //     map_2_odom_Trans.transform.rotation.w = map_to_odom.w();


    //     map_2_odom_Trans.header.stamp = gnss.header.stamp;
    //     tfBroadcasterMap2Odom.sendTransform(map_2_odom_Trans);
    // }

    void pubMap2OdomTrans()
    {
        double head = 0;
        while (!nh.getParam("/gnss/head", head))
        {
            ros::Duration(0.01).sleep();
        }

        double yaw = (90 - head) / 180 * M_PI;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        map_2_odom_trans.transform.rotation.x = quat.x();
        map_2_odom_trans.transform.rotation.y = quat.y();
        map_2_odom_trans.transform.rotation.z = quat.z();
        map_2_odom_trans.transform.rotation.w = quat.w();
        tfBroadcasterMap2Odom.sendTransform(map_2_odom_trans);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lego_loam");

    TransformFusion TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ros::spin();

    return 0;
}
