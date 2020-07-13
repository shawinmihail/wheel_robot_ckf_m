clc
clear
close all

%% read
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



%% pos process
% r
r0 = [rx0(1);ry0(1);rz0(1)];
[rx0_loc, ry0_loc, rz0_loc] = wgs_cartesian_to_local(r0, rx0, ry0, rz0);
[rx1_loc, ry1_loc, rz1_loc] = wgs_cartesian_to_local(r0, rx0-rx1, ry0-ry1, rz0-rz1);
[rx2_loc, ry2_loc, rz2_loc] = wgs_cartesian_to_local(r0, rx0-rx2, ry0-ry2, rz0-rz2);


% figure
% hold on
% plot3(rx0_loc, ry0_loc, rz0_loc)
% plot3(rx1_loc, ry1_loc, rz1_loc)
% plot3(rx2_loc, ry2_loc, rz2_loc)

% figure
% hold on
% for i = 30:160
%     plot3(rx0_loc(1:i), ry0_loc(1:i), rz0_loc(1:i), 'k');
%     plot3(rx1_loc(1:i), ry1_loc(1:i), rz1_loc(1:i), 'r');
%     plot3(rx2_loc(1:i), ry2_loc(1:i), rz2_loc(1:i), 'g');
%     pause(0.3)
% end

% v
[vx0_loc, vy0_loc, vz0_loc] = vel_from_pos(rx0_loc, ry0_loc, rz0_loc, t_gps);


%% drs gnss
figure
hold on
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
ex = [1;0;0];

for i = 30:60
    if ~(k1(i) > 3) | ~(k2(i) > 3)
        continue
    end
    v_loc = [vx0_loc(i), vy0_loc(i), vz0_loc(i)];
    dir = v_loc / norm(v_loc);
    
    dr1 = [rx1_loc(i)-rx0_loc(i); ry1_loc(i)-ry0_loc(i); rz1_loc(i)-rz0_loc(i)];
    dr2 = [rx2_loc(i)-rx0_loc(i); ry2_loc(i)-ry0_loc(i); rz2_loc(i)-rz0_loc(i)];
    rd = (dr1 + dr2) / 2;
    
    q = quatBetweenVectors([rd(1); rd(2); 0], ex);

    p1 = quatRotate(q, dr1)
    p2 = quatRotate(q, dr2)
    pd = quatRotate(q, rd);
    
%     %%
%     pin = pd/norm(pd);
%     alpha = -0.02;
%     q = [cos(alpha/2); pin * sin(alpha/2)];
%     p1 = quatRotate(q, p1)
%     p2 = quatRotate(q, p2)
    
    
%     plot3(rx0_loc(i), ry0_loc(i), rz0_loc(i), 'r*')
%     plot3(dr1(1), dr1(2), dr1(3), 'g*')
%     plot3(dr2(1), dr2(2), dr2(3), 'b*')
%     plot3(rd(1), rd(2), rd(3), 'k*')

    plot3(0, 0, 0, 'r*')
    plot3(p1(1), p1(2), p1(3), 'g*')
    plot3(p2(1), p2(2), p2(3), 'b*')
    plot3(pd(1), pd(2), pd(3), 'k*')
    p1s(:,i) = p1;
    p2s(:,i) = p2;
end

%%
% figure
% hold on
% ax = smooth(ax);
% ay = smooth(ay);
% az1 = smooth(az);
% plot(ax, 'r')
% plot(ay, 'g')
% plot(az, 'b')

 
% figure
% hold on
% plot(wx, 'r')
% plot(wy, 'g')
% plot(wz, 'b')

% figure
% hold on
% grid on
% plot3(rx0, ry0, rz0, 'r')
% plot3(rx0 + rx1, ry0 + ry1, rz0 + rz1, 'g')
% plot3(rx0 + rx2, ry0 + ry2, rz0 + rz2, 'b')

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

%% save
 
dr1 = [rx1_loc-rx0_loc, ry1_loc-ry0_loc, rz1_loc-rz0_loc];
dr2 = [rx2_loc-rx0_loc, ry2_loc-ry0_loc, rz2_loc-rz0_loc];

gps_mes = [t_gps, rx0_loc, ry0_loc, rz0_loc, vx0_loc, vy0_loc, vz0_loc, dr1, dr2, k0, k1, k2];
imu_mes = [t_imu, ax, ay, az, wx, wy, wz];


save('logs/gps_mes0', 'gps_mes')
save('logs/imu_mes0', 'imu_mes')