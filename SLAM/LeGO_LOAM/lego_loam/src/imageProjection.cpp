#include "lego_loam/utility.h"


class ImageProjection
{
private:
    ros::NodeHandle nh;

    std::string pointCloudTopic;
    ros::Subscriber subLaserCloud;

    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubOutlierCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudInfo;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud;     // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat;  // range matrix for range image
    cv::Mat labelMat;  // label matrix for segmentaiton marking
                       // 值如果是-1，则说明这个点是地面点或者是无效点（特指是硬件没有数据的情况）
                       // 值如果是0，则说明暂时还没有处理这一个点，接下来会继续进行点云分割
                       // 值如果是999999，则说明这个点已经被尝试进行了点云分割，但不满足是一个聚类的要求，认为是无效点
                       // 值如果是其他正数，则说明这个点已经分割好了，并且数值表明了这个点是属于哪一个点云聚类的编号
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t>> neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

public:
    ImageProjection() : nh("~")
    {
        pointCloudTopic = nh.param<std::string>("point_cloud_topic", "/static_point_ros");
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this); // 订阅原始的激光点云

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1); // 转换成图片的投影点云，带有行列信息
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);  // 转换成图片的投影点云，带有距离信息

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);                // 地面点云
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);              // 非地面点非聚类点云
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);          // 包含地面点的聚类点云
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);   // 上述点云的辅助信息
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1); // 主要用于可视化的非地面点聚类点云

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    ~ImageProjection() {}

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN * Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN * Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN * Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN * Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1;
        neighbor.second = 0;
        neighborIterator.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = 1;
        neighborIterator.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = -1;
        neighborIterator.push_back(neighbor);
        neighbor.first = 1;
        neighbor.second = 0;
        neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    // 订阅激光雷达点云信息之后的回调函数
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        ROS_INFO("lego_loam -> imagePrjection -> cloudHandler !");
        // 1. Convert ros message to pcl point cloud
        // 1. 把 ros 格式的点云转换并存储成 pcl 格式点云
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        // 2. 找到开始时刻和结束时刻的方向角度
        findStartEndAngle();
        // 3. Range image projection
        // 3.将点云信息投影到 16 * 1800 分辨率的图像上（点云阵列上）
        projectPointCloud();
        // 4. Mark ground points
        // 4. 根据上下线束 俯仰角 判断是否是 地面  （角度小于10度 为地面）
        groundRemoval();
        // 5. Point cloud segmentation
        // 5. 点云分割 首先对点云进行聚类标记 然后通过聚类完成的标签 对点云分块存储
        cloudSegmentation();
        // 6. Publish all clouds
        // 6. Publish all clouds 发布各类型的点云
        publishCloud();
        // 7. Reset parameters for next iteration
        // 7. 重启
        resetParameters();
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // 1. 读取ROS点云转换为PCL点云
        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        // 2.移除无效的点云 Remove Nan points
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        // 3. have "ring" channel in the cloud or not
        // 如果点云有"ring"通过，则保存为laserCloudInRing
        // 判断是不是使用了 velodyne 的激光雷达
        if (useCloudRing == true)
        {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
    }

    void findStartEndAngle()
    {
        // 1.开始点和结束点的航向角 (负号表示顺时针旋转)
        // start and end orientation of this cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                       laserCloudIn->points[laserCloudIn->points.size() - 1].x) +
                                2 * M_PI; // 加 2 * M_PI 表示已经转转了一圈

        // 2.保证 所有角度 落在 [M_PI , 3M_PI] 上
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI)
        {
            segMsg.endOrientation -= 2 * M_PI;
        }
        else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
        {
            segMsg.endOrientation += 2 * M_PI;
        }
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    void projectPointCloud()
    {
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i)
        {
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            if (useCloudRing == true) // 判断是不是使用了 velodyne 的雷达
            {
                rowIdn = laserCloudInRing->points[i].ring; // 提取激光雷达线束到 rowIdn
            }
            else // 是其他的雷达 就通过俯仰角 确定当前的激光点是来自哪个线束 index
            {
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;

            // 将计算下来的距离传感器的 数值保存到 rangeMat 中
            // 这是一个 16 * 1800 的矩阵 rowIdn为线束数值  columnIdn 是 一圈圆形 滩平之后的数值
            // range  是特征点云点到原点的数值
            // 这样就将一个三维的坐标 转换到一个 矩阵中了.
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 将 index 和 横坐标存储在 intensity 中 整数部分是线束数值  小数部分是方向角度
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            // 深度图的索引值  index = 列号 +  行号 * 1800
            index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

    void groundRemoval()
    {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not 没有有效的信息确认是不是地面
        //  0, initial value, after validation, means not ground 确认不是地面点
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
            //  前7个激光雷达扫描线束足够满足地面点的检测 所以只遍历 7 次。而且前 7 线是从下往上数的
            for (size_t i = 0; i < groundScanInd; ++i)
            {
                lowerInd = j + (i)*Horizon_SCAN;
                upperInd = j + (i + 1) * Horizon_SCAN;

                // 如果之前计算过的 intensity 是 -1 则直接默认为是一个无效点
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1)
                {
                    // no info to check, invalid points 对这个无效点直接进行贴标签
                    groundMat.at<int8_t>(i, j) = -1;
                    continue;
                }

                // 计算相邻两个线束之间的夹角
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

                // 如果夹角数值小于 10 度， 则可以判断为平面
                if (abs(angle - sensorMountAngle) <= 10)
                {
                    groundMat.at<int8_t>(i, j) = 1;
                    groundMat.at<int8_t>(i + 1, j) = 1;
                }
            }
        }

        // 给 地面点和未测量到的点 标记一个符号 为 -1，被标记的点不参与后续的 segmentation 计算
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i)
        {
            for (size_t j = 0; j < Horizon_SCAN; ++j)
            {
                if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX)
                {
                    labelMat.at<int>(i, j) = -1;
                }
            }
        }

        // 组织地面点云信息
        if (pubGroundCloud.getNumSubscribers() != 0)
        {
            for (size_t i = 0; i <= groundScanInd; ++i)
            {
                for (size_t j = 0; j < Horizon_SCAN; ++j)
                {
                    if (groundMat.at<int8_t>(i, j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation()
    {
        // segmentation process
        // 以每个点为起始，将所有点进行聚类分割，根据分割结果打上标签
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i, j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry 提取分割点云 用于激光雷达里程计
        for (size_t i = 0; i < N_SCAN; ++i)
        {
            segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j)
            {
                if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1)
                {
                    // outliers that will not be used for optimization (always continue)
                    // 勾勒出优化过程中不被使用的值

                    // 1. 如果label为999999则跳过，也就是不属于聚类的点跳过
                    if (labelMat.at<int>(i, j) == 999999)
                    {
                        if (i > groundScanInd && j % 5 == 0)
                        {
                            outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                            continue;
                        }
                        else
                        {
                            continue;
                        }
                    }

                    // 2. 如果为地，跳过index不是5的倍数的点
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i, j) == 1)
                    {
                        if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
                            continue;
                    }

                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
        }

        // 用于可视化，将每一个聚类的 intensity 都设置为对应的编号，这样可视化每一组聚类都有不同的颜色
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0)
        {
            for (size_t i = 0; i < N_SCAN; ++i)
            {
                for (size_t j = 0; j < Horizon_SCAN; ++j)
                {
                    if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999)
                    {
                        segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col)
    {
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        // 传进来的两个参数，按照坐标不同 分别给他们放到 X 与 Y 的数组中
        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1; // 需要计算角度的点的数量
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while (queueSize > 0)
        {
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            // 遍历整个点云 遍历上下左右四个点,求点之间的角度数值
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
            {
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0) // 不对已经分类好的点云再次进行分割，相当于每次while循环都将会尝试分割出一个点云聚类，然后再次聚类时就跳过这些已经分割好的点云
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

                // 当β>临界值时，就判定这两个点属于同一个物体。
                if (angle > segmentTheta)
                {
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;

        // 如果是 allPushedIndSize 累加的数量增加到了30个，则判断这部分点云属于一个聚类
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        // 或者垂直方向上有 属于这个聚类的点 的线数的数量大于5个，则也可以认为这是一个有效的聚类
        else if (allPushedIndSize >= segmentValidPointNum)
        {
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }

        // segment is valid, mark these points
        if (feasibleSegment == true)
        {
            ++labelCount;
        }
        else // segment is invalid, mark these points 无效的点，则标号999999
        {
            for (size_t i = 0; i < allPushedIndSize; ++i)
            {
                // 其实也就是把刚刚暂时认为是一个聚类的点云的标签值（可能已经被设置成了1，2，3等）更改成是无效的编号999999
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void publishCloud()
    {
        // 1. 分割后的点云辅助信息，描述的点是2.5中的点。具体信息包含了按照每一线存储的点云的起始和结束位置（便于定位每一线点云的位置），也包含了每个点的辅助信息
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);

        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;
        // 2.1 投影点云。
        // 所有点XYZI，16*1800，包括nan点，intensity代表行列信息
        if (pubFullCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "rslidar_link"; // 原始为“base_link”
            pubFullCloud.publish(laserCloudTemp);
        }
        // 2.2 投影点云。
        // 所有点XYZI，16*1800，包括nan点，intensity代表点的距离
        if (pubFullInfoCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "rslidar_link"; // 原始为“base_link”
            pubFullInfoCloud.publish(laserCloudTemp);
        }
        // 2.3 地面点云。
        // 所有的地面点云XYZI，未经过下采样。
        if (pubGroundCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "rslidar_link"; // 原始为“base_link”
            pubGroundCloud.publish(laserCloudTemp);
        }
        // 2.4 发布异常信息的点云。
        // 具体就是非地面点中，没有构成聚类的点的下采样点
        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "rslidar_link"; // 原始为“base_link”
        pubOutlierCloud.publish(laserCloudTemp);
        // 2.5 分割后的点云。
        // 具体包含 所有合规的聚类点 和 地面点的下采样点两种点。这些点是按照 scan 顺序加入的，不包含聚类具体属于哪一个聚类的信息
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "rslidar_link"; // 原始为“base_link”
        pubSegmentedCloud.publish(laserCloudTemp);
        // 2.6 非地面点的聚类点云。
        // 主要用于可视化的聚类点云显示。所有的聚类点，不包含地面点，不同的聚类具有不同的intensity值，可视化中有不同的颜色。
        if (pubSegmentedCloudPure.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "rslidar_link"; // 原始为“base_link”
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lego_loam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
