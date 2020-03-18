clc
clear
close all

%% cleared imu
path = 'logs/example2/';
name_base = 'uav_control_2020_03_11_';
num = '0';

type_imu = 'imu';
type_gps = 'pos';
path_imu_data = [path name_base num '_' type_imu '.log'];
path_gps_data = [path name_base num '_' type_gps '.log'];
imu_data_table = readtable(path_imu_data, 'FileType', 'text');
gps_data_table = readtable(path_gps_data, 'FileType', 'text');

%%
t0 = imu_data_table.Var2(1);
t_imu = (imu_data_table.Var2 - t0) / 10^9;
ax = imu_data_table.Var4;
ay = imu_data_table.Var5;
az = imu_data_table.Var6;
wx = imu_data_table.Var7;
wy = imu_data_table.Var8;
wz = imu_data_table.Var9;

%%
t_gps = (gps_data_table.Var2 - t0) / 10^9;
k0 = gps_data_table.Var3;
rx0 = gps_data_table.Var4;
ry0 = gps_data_table.Var5;
rz0 = gps_data_table.Var6;

k1 = gps_data_table.Var7;
rx1 = gps_data_table.Var8;
ry1 = gps_data_table.Var9;
rz1 = gps_data_table.Var10;

k2 = gps_data_table.Var11;
rx2 = gps_data_table.Var12;
ry2 = gps_data_table.Var13;
rz2 = gps_data_table.Var14;

imu_data = [t_imu ax ay az wx wy wz];
gps_data = [t_gps rx0 ry0 rz0 rx1 ry1 rz1 rx2 ry2 rz2];

%% v
% figure
% hold on
% 
% dt_gps = diff(t_gps);
% vx = diff(rx0) ./ dt_gps;
% 
% vlim = 1.2;
% vx(vx>vlim) = vlim;
% vx(vx<-vlim) = -vlim;
% plot(vx)
% plot(smooth(vx), 'k')

%% drs gnss
figure
hold on
ex = [1;0;0];

for i = 1:60
    if ~(k1(i) > 3) | ~(k2(i) > 3)
        continue
    end
    r1 = [rx1(i), ry1(i), rz1(i)]';
    r2 = [rx2(i), ry2(i), rz2(i)]';
    rd = r1 + (r2 - r1) / 2;

    q = quatBetweenVectors(rd, ex);

    p1 = quatRotate(q, r1);
    p2 = quatRotate(q, r2);
    pd = quatRotate(q, rd);
    
%     plot3(0, 0, 0, 'r*')
%     plot3(r1(1), r1(2), r1(3), 'g*')
%     plot3(r2(1), r2(2), r2(3), 'b*')
%     plot3(rd(1), rd(2), rd(3), 'k*')

    plot3(0, 0, 0, 'r*')
    plot3(p1(1), p1(2), p1(3), 'g*')
    plot3(p2(1), p2(2), p2(3), 'b*')
    plot3(pd(1), pd(2), pd(3), 'k*')
end


% figure
% hold on
% plot(ax, 'r')
% plot(ay, 'g')
% plot(az, 'b')
% 
% figure
% hold on
% plot(wx, 'r')
% plot(wy, 'g')
% plot(wz, 'b')

% figure
% hold on
% grid on
% plot3(rx0-rx0(1), ry0-ry0(1), rz0-rz0(1), 'r')
% plot3(rx0-rx0(1) + rx1, ry0-ry0(1) + ry1, rz0-rz0(1) + rz1, 'g')
% plot3(rx0-rx0(1) + rx2, ry0-ry0(1) + ry2, rz0-rz0(1) + rz2, 'b')

% figure
% hold on
% plot(rx0-rx0(1), 'r')
% plot(ry0-ry0(1), 'g')
% plot(rz0-rz0(1), 'b')
% 
% figure
% hold on
% plot(vx, 'r')
% plot(vy, 'g')
% plot(vz, 'b') 