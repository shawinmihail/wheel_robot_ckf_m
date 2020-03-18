clc
clear
close all

%% cleared imu
path = 'logs/example2/';
name_base = 'uav_control_2020_03_11_';
num = '10';

type_imu = 'imu';
type_gps = 'pos';
path_imu_data = [path name_base num '_' type_imu '.log'];
path_gps_data = [path name_base num '_' type_gps '.log'];
imu_data_table = readtable(path_imu_data, 'FileType', 'text');
gps_data_table = readtable(path_gps_data, 'FileType', 'text');


ax = imu_data_table.Var4;
ay = imu_data_table.Var5;
az = imu_data_table.Var6;
ax_cl = imu_data_table.Var7;
ay_cl = imu_data_table.Var8;
az_cl = imu_data_table.Var9;

rx = gps_data_table.Var4;
ry = gps_data_table.Var5;
rz = gps_data_table.Var6;
vx = gps_data_table.Var7;
vy = gps_data_table.Var8;
vz = gps_data_table.Var9;

figure
hold on
plot(ax, 'r')
plot(ay, 'g')
plot(az, 'b')

figure
hold on
plot(ax_cl, 'r')
plot(ay_cl, 'g')
plot(az_cl, 'b')

figure
hold on
grid on
plot3(rx-rx(1), ry-ry(1), rz-rz(1), 'r')

figure
hold on
plot(rx-rx(1), 'r')
plot(ry-ry(1), 'g')
plot(rz-rz(1), 'b')

figure
hold on
plot(vx, 'r')
plot(vy, 'g')
plot(vz, 'b') 