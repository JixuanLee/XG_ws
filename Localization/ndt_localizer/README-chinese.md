一个简洁而优雅的NDT定位器

这个仓库是从Autoware的lidar_localizer模块修改而来的。不同于Autoware中的模块，它依赖于很多的包（你需要编译Autoware项目中的所有的包），这个仓库是干净，简单，而且没有依赖。你所需要的只是ROS，和一个pcd文件（点云地图）。

让我们从这个简单的仓库开始，学习基于激光雷达的定位吧！

在点云地图（pcd）中进行定位
cfgs/ndt_result.gif

一个在MulRan数据集上的演示视频：

<a href="https://img.youtube.com/vi/qhqDmmO7c4c/0.jpg">![IMAGE ALT TEXT HERE</a>](<a href="https://www.youtube.com/watch?v=qhqDmmO7c4c">https://www.youtube.com/watch?v=qhqDmmO7c4c</a>)

如何使用
准备你的pcd地图和rosbag
你可以复现我的博客 <a href="https://blog.csdn.net/AdamShan/article/details/106739856?spm=1001.2014.3001.5501">基于NDT的自动驾驶高精度定位和ROS项目实战</a> 和 <a href="https://blog.csdn.net/AdamShan/article/details/106589633?spm=1001.2014.3001.5501">使用SC-LEGO-LOAM进行较大规模点云地图构建和闭环优化</a> 来使用Mulran数据集来构建你的pcd地图和生成点云数据。如果你不能阅读中文博客，而又想复现项目演示，你可以使用下面的链接（百度网盘）来下载pcd地图和rosbag：

链接: <a href="https://pan.baidu.com/s/1hZ0VuQCy4KX3lHUTFdVeww">https://pan.baidu.com/s/1hZ0VuQCy4KX3lHUTFdVeww</a>  密码: r7fl

cfgs/4.png

> KAIST02-small.bag不是整个KAIST02数据集，因为rosbag不压缩数据，整个KAIST02 rosbag太大了。所以我使用了KAIST02数据集的前81秒来制作这个小的rosbag。

将pcd数据放到map文件夹中：

bash
cp kaist02.pcd map/

在你的ros工作空间中编译
在你的ros工作空间/src/中克隆这个仓库，然后catkin_make（或者catkin build）：
bash
cd catkin_ws/src/
git clone https://github.com/AbangLZU/ndt_localizer.git
cd ..
catkin_make

设置配置
配置地图加载器
将你的地图pcd文件（.pcd）放到这个项目中的map文件夹中（ndt_localizer/map），在map_loader.launch中修改pcd_path为你的pcd路径，例如：

xml
<arg name="pcd_path"  default="$(find ndt_localizer)/map/kaist02.pcd"/>

配置点云下采样
在launch/points_downsample.launch中配置你的激光雷达点云话题：

xml
<arg name="points_topic" default="/os1_points" />

如果你的激光雷达数据是稀疏的（比如VLP-16），你需要在launch/points_downsample.launch中配置更小的leaf_size，比如2.0。如果你的激光雷达点云是密集的（VLP-32, Hesai Pander40P, HDL-64等），保持leaf_size为3.0。

配置静态tf
这个项目中有两个静态变换：base_link_to_localizer和world_to_map，如果你使用的是不同的激光雷达，用你的激光雷达的坐标系id替换ouster：

xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_localizer" args="0 0 0 0 0 0 base_link ouster"/>

配置ndt定位器
你可以在ndt_localizer.launch中配置NDT的参数。NDT算法的主要参数是：

xml
<arg name="trans_epsilon" default="0.05" doc="The maximum difference between two consecutive transformations in order to consider convergence" />
<arg name="step_size" default="0.1" doc="The newton line search maximum step length" />
<arg name="resolution" default="2.0" doc="The ND voxel grid resolution" />
<arg name="max_iterations" default="30.0" doc="The number of iterations required to calculate alignment" />
<arg name="converged_param_transform_probability" default="3.0" doc="" />

这些默认的参数对于64和32线的激光雷达都很好用。

运行定位器
一旦你准备好了你的pcd地图和配置，就可以运行定位器了：

bash

打开一个roscore
roscore

在另一个终端
cd catkin_ws
source devel/setup.bash

如果你是在播放一个rosbag，使用rosbag的模拟时间！！！
rosparam set use_sim_time true

启动ndt_localizer节点
roslaunch ndt_localizer ndt_localizer.launch

等待几秒钟加载地图，然后你可以在rviz中看到你的pcd地图，像这样：

cfgs/sample_img_1.png

用rviz中的2D Pose Estimate给出当前车辆的初始位姿：

cfgs/sample_img3.png

这个操作会向话题/initialpose发送一个初始位姿。

播放rosbag：

bash
rosbag play KAIST02-small.bag --clock

然后你就可以看到定位的结果了：

cfgs/sample_img2.png

最终的定位消息会发送到/ndt_pose话题：

proto
header:
seq: 1867
stamp:
secs: 1566536121
nsecs: 251423898
frame_id: "map"
pose:
position:
x: -94.8022766113
y: 544.097351074
z: 42.5747337341
orientation:
x: 0.0243843578881
y: 0.0533175268768
z: -0.702325920272
w: 0.709437048124
定位器也会发布一个base_link到map的tf：

transforms:
•

header:
seq: 0
stamp:
secs: 1566536121
nsecs: 251423898
frame_id: "map"
child_frame_id: "base_link"
transform:
translation:
x: -94.8022766113
y: 544.097351074
z: 42.5747337341
rotation:
x: 0.0243843578881
y: 0.0533175268768
z: -0.702325920272
w: 0.709437048124

想了解更多细节吗？
你可以关注我
