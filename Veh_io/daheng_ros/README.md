# daheng_ros_driver使用说明

1. 先到官网下载所用相机对应的sdk
目前使用的Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.5.2303.9221版本，解压后在终端运行
```sudo ./Galaxy_camera.run```
完成官方上位机和驱动的安装

2. 使用对应的ros包
大恒官方不提供ros包，使用第三方的ros包。但这个第三方ros包依赖于rm_control包，因此需要先配置rm_control包和相关的依赖。  
此ros包的CmakeLists已经重写，完全不需要修改，根据相对路径已经完成相关配置。

