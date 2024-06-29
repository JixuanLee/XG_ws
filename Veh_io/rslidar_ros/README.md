rslidar_sdk-v1.5.10 from github
以下均是对RS用户手册、github源码附带README.md、自己相关使用经验（坑）的精华总结，凡未寻得的内容，详见用户手册or官方README.md。

适配RS-Helios-1615(32线，型号识别码RS-Helios) 与 RS-Helios-16P(16线，型号识别码RS-Helios)。

源码编译（ROS1系统）请参考“4.2 依赖于ROS-catkin编译”项（源码README.md）
	编译失败：若提示yaml库找不到定义，把/usr/local/include/的yaml-cpp文件夹迁移至/usr/include/即可。

连接RS雷达实物前，需要到web端进行IP设置与参数设置。RS各型号雷达出厂默认IP均192.168.1.200。因此，默认web地址：192.168.1.200/。设置方式：首先连接RS雷达至主机（按照默认IP网口连接0），使用主机浏览器方位上述默认web地址，进行修改。修改参数的中文注释参考RS雷达设备用户手册P34-P37。

雷达连接：安装下方ip及端口连接。

使用前必须根据实际情况修改/config/config.yaml：
1.如果多个雷达都连接启动，在其中的“lidar:”后面整齐排列“  - drive:.....   ros:...   -drive:....   ros:... ”，依次列出对应雷达的参数即可。另外，需要注意将多个雷达的端口号设置成不一样的。 
2.
3.其他参数按需修改，相关注释已经手动加入config.yaml
4.注意严格缩进和格式书写



在线进行雷达数据捕捉
	设置config.yaml模式为1
	启动：
	source devel/setup.bash
	roslaunch rslidar_sdk start.launch
在线录制雷达Packet包
	设置config.yaml模式为1，改变：send_packet_ros: true，按需修改ros_send_packet_topic（包括MSOP Packet和DIFOP Packet）
	a.录制ROSbag：rosbag record /rslidar_packets -O bag
	b.官网没写，之后有需要再补充
离线解析现有雷达数据
	设置config.yaml模式为2或3，设置数据包的绝对路径
	模式2/ROS包：
	模式3/Pcap包：
	即便Pcap是多个 LiDAR录制得到的，但只要这些LiDAR使用同一目的IP、端口不同，就可以按照普通的msop_port:6699、difop_port:7788来配置config.yaml（本README.md x.y所述即为此方法）。但是最好仅用1个LiDAR录制包。
	source devel/setup.bash
	roslaunch rslidar_sdk start.launch




注意！！！！！
front雷达是RS16，需要上位机进行IP设定等。

