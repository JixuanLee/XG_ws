# 1.rosbag 转 pcd：

## 1.0 说明
无论通过下述哪类转换，得到的pcd文件均是离散的、以时间戳为名的、一坨文件。他们分别是某一帧/时刻下，对应的点云话题的所有数据。
因此，如果将所有的离散pcd文件整合为1个，本质上就是所有时刻的点云数据的叠加，将会呈现处该点云topic在该段时间内所有的连续点云变换情况。

## 1.1 离线rosbag转pcd，参数：bag路径、目标话题（似乎只能写一个话题）、pcd输出文件夹、目标坐标系（可选）
rosrun pcl_ros bag_to_pcd /home/vtie/Lidar_AC_BIT_room.bag /rslidar_points_16A /home/vtie/Lidar_A_pcd /rslidar_16A

## 1.2 在线pointcloud转pcd
rosrun pcl_ros pointcloud_to_pcd input:=/rslidar_points_16C

# 2.离散pcd合并

cd /home/vtie/XG_ws
source ./devel/setup.bash
模板：roslaunch lidar_preprocess pcd_merge.launch pcd_dir:=你的pcd们的文件夹 pcd_out:=你输出的目标文件夹/文件名.pcd
实例：roslaunch lidar_preprocess pcd_merge.launch pcd_in_dir:=/home/vtie/pcds_file/ pcd_out_dir:=/home/vtie/pcd_out.pcd

合并后将得到1个完整的pcd，记录一段时间内、目标点云话题下的点云数据的、相对连续的变化情况。

注意：合并文件的时间很长很长。100个pcd大概需要5-10s。合并前后，总的文件的大小没有太大变化。

# 3.pcd文件展示

## 3.1调用pcl_viewer包
这是一个pcl自带的简易rviz类型软件。
cd /home/vtie/
模板：pcl_viewer 目标pcd文件名1.pcd 目标pcd文件名2.pcd ... 
实例：pcl_viewer pcd_out1.pcd pcd_out2.pcd

注意：如果加载的pcd文件点云数量过多，可能会非常卡。。

## 3.2调用PCDViewer包（推荐！）
这是一个成熟且仍在优化的开源PCD处理工具，功能非常强大，内嵌地面点滤除等各种功能。
启动终端，输入：PCDViewer
将pcd文件拖入。可以测量任意点云距离，适合标定。


# 4.点云格式转换流程
## 4.1 原始数据解包sdk
rs的sdk使用了自定义的点类型XYZI，其中intensity数据类型是uint8，而pcl的XYZI中的intensity数据类型是float，这就造成了如果
直接用pcl的XYZI数据格式去接受sdk解包出的点云，会由于intensity数据类型不同而报错：
```
Failed to find match for field 'intensity'.
```

## 4.2 点云拼接和预处理
四个激光雷达点云拼接时采用XYZ格式的点进行接收，相当于直接舍弃了intensity部分的数据。这过程不报错。

## 4.3 动态点滤除
激光雷达点云使用pcl的XYZI格式接收4.2中的点云，但其中的intensity没有赋值。如果原始的数据经过4.1、4.2、4.3后，就相当于在不报错的情况下，将rs的sdk的XYZI格式转换成了pcl的XYZI格式，只不过目前其中的intensity值为空。

## 4.4 SLAM建图
在SLAM建图中，需要使用pcl格式的XYZI点格式，这是因为其中的intensity需要用作存储激光雷达行列信息，整数部分和小数部分代表不同含义，因此需要float类型的XYZI格式。  
如果直接使用4.3中的数据，则不会报错，点的数据格式都是pcl的XYZI。  
但是直接使用4.1sdk的数据也只是会报错，不影响实际功能。因为uint8类型的intensity数据目前完全没有应用的需要，因此在转换到float类型过程中失败，uint8的intensity数据直接被丢弃并且报错是没有影响的，在slam后续的程序中会将有关行列信息赋值给float类型的intensity，之后再进行后续工作，因此是不影响实际功能的。