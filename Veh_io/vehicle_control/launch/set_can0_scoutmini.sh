#!/bin/bash

#密码为当前主机开机密码
echo "001210" | sudo -S modprobe gs_usb
echo "001210" | sudo -S ip link set can0 up type can bitrate 500000 # 徐工 250000

exit 0
