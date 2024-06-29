# 话题列表：

># 1.解包部分
话题                 | 数据类型                       | 作用
----                | -----                         | ------  
/cmd_raw            | std_msgs::Float32MultiArray   | 获得原始的控制命令  
                         
># 2.数据处理部分
话题                 | 数据类型                       | 作用
----                | -----                         | ------  
/cmd_scoutmini      | geometry_msgs::Twist          | 接收/cmd_raw中的数据，转换成控制scoutmini小车的命令      
/cmd_XUV25B         | geometry_msgs::Twist          | 接收/cmd_raw中的数据，转换成控制XUV25B车辆的命令      
