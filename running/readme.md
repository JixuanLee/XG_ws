# 功能包介绍
本空能包为最终运行接口，一键实现对目标实时跟随，跟随路径的效果。

## 0.第一次运行
### 0.1 首先进入工作空间，完成编译；配置can通线相关工具。
```
cd XG_ws  
catkin_make
sudo apt install -y can-utils
```
### 0.2 然后对相关脚本文件进行配置,对这些文件进行授权：
```
sudo chmod 777 src/uwb_unpack/launch/chmod_usb_anchor0.sh
sudo chmod 777 src/vehicle_control/launch/set_can0_scoutmini.sh
sudo chmod 777 src/vehicle_control/launch/set_can0_XUV25B.sh
sudo chmod 777 src/XG_autostart.sh
```
### 0.3 然后用下述命令打开文件后，将对应的密码改成本机密码。
```  
gedit src/uwb_unpack/launch/chmod_usb_anchor0.sh
gedit src/vehicle_control/launch/set_can0_scoutmini.sh
gedit src/vehicle_control/launch/set_can0_XUV25B.sh
```  
### 0.4 配置文件夹下程序自启动脚本:
```
gnome-session-properties
```
选择"add"添加脚本，"name"自定，建议`XG_autostart`，"command"选择
```
/home/usr/XG_ws/src/XG_autostart.sh
```
其中`usr`为当前用户名。  
### 0.5 将anchor0与电脑的硬件接口进行绑定
* 剔除所有usb设备，只保留必要的鼠标、键盘、屏幕的usb线，然后重启电脑；
* 只插上anchor0的usb线；
* 通过ls /dev命令确定设备名称，本套应该是(ttyACM0)；
* 通过以下命令查看ttyUSB0的KERNELS硬件端口号，后面的ACM0根据实际更改  
```
udevadm info --attribute-walk --name=/dev/ttyACM0
```
* 然后找到最子级的 KERNELS == "2-2.2:1.0"，其中 2-2.2:1.0 是对应的版本号；
* 通过以下命令创建规则文件，更改别名：
```
sudo gedit /etc/udev/rules.d/usb.rules
```
* 输入以下命令，其中后面的usb_anchor0即为要更改的名字
```
KERNELS=="2-2.2:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="usb_anchor0"
```
* 运行以下命令使修改立即生效:
```
sudo udevadm trigger 
```
* 在此之后就可以在程序中使用usb_anchor0实现打开对应的硬件接口了

## 1.运行可视化rviz
使用rviz进行可视化，其中已经进行了一定配置。
```
roslaunch running rviz.launch
```

## 2.运行目标点实时跟随
### 2.1如果使用XUV25B车辆：
每次连接好can线之后，需要执行该命令一次：
```
src/vehicle_control/launch/set_can0_XUV25B.sh
```
然后正常运行：
```
roslaunch running follow_XUV25B.launch
```
### 2.2如果使用scoutmini小车：
每次连接好can线之后，需要执行该命令一次：
```
src/vehicle_control/launch/set_can0_scoutmini.sh
```
然后正常运行：
```
roslaunch running follow_scoutmini.launch
```
### 2.3注意
上述launch文件中包含是否需要对运动过程点进行本地保存的命令，根据需要进行注释。

## 3.运行轨迹跟踪效果
运行前需要在对应launch文件处进行修改，选择模式mode为”file“读文件或者”uwb“读取uwb数据。  
### 3.1如果使用XUV25B车辆：
每次连接好can线之后，需要执行该命令一次：
```
src/vehicle_control/launch/set_can0_XUV25B.sh
```
```
roslaunch running tracking_XUV25B.launch
```
### 3.2如果使用scoutmini小车：
每次连接好can线之后，需要执行该命令一次：
```
src/vehicle_control/launch/set_can0_scoutmini.sh
```
然后正常运行：
```
roslaunch running tracking_scoutmini.launch
```