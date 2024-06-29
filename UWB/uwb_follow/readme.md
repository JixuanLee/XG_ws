# 功能包介绍

## 1.代码介绍
本功能包利用uwb传来的数据对tag进行定位,然后根据当前位置实时跟随目标.然后根据车辆目前位置与tag位置关系进行pid控制并修正,生成用于控制车辆的线速度linear和角速度angular,然后以话题/cmd_raw的形式发布.  

具体的控制逻辑如下：

目标与车辆之间角度theta                    | 当前设定角度deg         | 车辆下发状态    
----                                    | -----                 | ------  
fabs(theta) > Angle_reverse             | fabs(theta) > 88      | pid控制，线速度再乘以衰退系数。当线速度小于阈值时停车   
fabs(theta) < Angle_reverse             | fabs(theta) < 88      | 车辆正常运动，具体运动状态看下表   


目标与车辆之间距离distance                 | 当前设定距离m           | 车辆下发状态    
----                                    | -----                 | ------  
distance > Dis_stop                     | distance > 6          | pid控制，车辆向目标点前进  
Dis_brake < distance < Dis_stop         | 3.5 < distance < 6    | pid控制，线速度再乘以衰退系数。当线速度小于阈值时停车  
Dis_rear < distance < Dis_brake         | 3 < distance < 3.5    | 运动死区，强制刹车停止  
distance < Dis_rear                     | distance < 3          | 恒定线速度，0角速度，倒车  
  
过程中用到的话题可参考[topiclist.md](topiclist.md)
