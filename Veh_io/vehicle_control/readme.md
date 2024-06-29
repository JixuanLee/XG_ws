# 功能包介绍

## 0.第一次运行程序需要授权!!
对[set_can0_scoutmini.sh](launch/set_can0_scoutmini.sh)和[set_can0_XUV25B.sh](launch/set_can0_XUV25B.sh)
0.1 用下述命令打开本功能包的launch文件夹中的.sh文件，将其中的密码设置为本机的开机密码。 
`gedit src/vehicle_control/launch/set_can0_scoutmini.sh`   
`gedit src/vehicle_control/launch/set_can0_XUV25B.sh`   
0.2 给文件进行授权,输入  
`sudo chmod 777 src/vehicle_control/launch/set_can0_scoutmini.sh`
`sudo chmod 777 src/vehicle_control/launch/set_can0_XUV25B.sh`

## 1.代码介绍
本功能包实现将原始控制命令/cmd_raw转换成对应车辆所需要的数据格式和话题。
  
过程中用到的话题可参考[topiclist.md](topiclist.md)
