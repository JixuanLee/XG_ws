clear all
clc

imugnss = readtable('imu_gnss.csv');
cmd = readtable('cmd.csv');

figure(1)
plot(imugnss.timestamp-imugnss.timestamp(1), imugnss.v_x, "r-")
hold on
plot(cmd.timestamp-cmd.timestamp(1), cmd.v_x, "b-")
legend('imugnss', 'cmd')
title('v_x')

figure(2)
plot(imugnss.timestamp-imugnss.timestamp(1), imugnss.w_z, "r-")
hold on
plot(cmd.timestamp-cmd.timestamp(1), cmd.w_z, "b-")
legend('imugnss', 'cmd')
title('w_z')