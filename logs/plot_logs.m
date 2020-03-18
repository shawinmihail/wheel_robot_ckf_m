clc
clear
close all

%% cleared imu
path_imu_data = 'logs/example2/uav_control_2020_02_11_7_imu.log';
data_table = readtable(path_imu_data, 'FileType', 'text');

n = length(data_table.Var4)
for i = 1:n
    str_rx = data_table.Var4(i);
    str_rx = str_rx{1};
    str_rx = str_rx(1:end-1);
    ax(i) = str2num(str_rx);
    
    str_ay = data_table.Var5(i);
    str_ay = str_ay{1};
    str_ay = str_ay(1:end-1);
    ay(i) = str2num(str_ay);

    str_rz = data_table.Var6(i);
    az(i) = str_rz;
    
    str_Ax = data_table.Var7(i);
    str_Ax = str_Ax{1};
    str_Ax = str_Ax(1:end-1);
    wx(i) = str2num(str_Ax);
     
    str_Ay = data_table.Var8(i);
    str_Ay = str_Ay{1};
    str_Ay = str_Ay(1:end-1);
    wy(i) = str2num(str_Ay);
 
    str_Az = data_table.Var9(i);
    wz(i) = str_Az;
end

rms_ax = rms(ax - mean(ax))
rms_ay = rms(ay - mean(ay))
rms_az = rms(az - mean(az))

rms_wx = rms(wx - mean(wx))
rms_wy = rms(wy - mean(wy))
rms_wz = rms(wz - mean(wz))

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
return
%% raw imu
path_imu_data = 'logs/example1/uav_control_2020_02_11_7_raw_imu.log';
data_table = readtable(path_imu_data, 'FileType', 'text');

n = length(data_table.Var4)
for i = 1:n
    str_rx = data_table.Var4(i);
    str_rx = str_rx{1};
    str_rx = str_rx(1:end-1);
    ax(i) = str2num(str_rx);
    
    str_ay = data_table.Var5(i);
    str_ay = str_ay{1};
    str_ay = str_ay(1:end-1);
    ay(i) = str2num(str_ay);

    str_rz = data_table.Var6(i);
    az(i) = str_rz;
    
    str_Ax = data_table.Var7(i);
    str_Ax = str_Ax{1};
    str_Ax = str_Ax(1:end-1);
    wx(i) = str2num(str_Ax);
     
    str_Ay = data_table.Var8(i);
    str_Ay = str_Ay{1};
    str_Ay = str_Ay(1:end-1);
    wy(i) = str2num(str_Ay);
 
    str_Az = data_table.Var9(i);
    wz(i) = str_Az;
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

%% pos vel
path_imu_data = 'logs/example1/uav_control_2020_02_11_7_position.log';
data_table = readtable(path_imu_data, 'FileType', 'text');

n = length(data_table.Var4)
for i = 1:n
    str_k1(i) = data_table.Var3(i);
    
    str_rx = data_table.Var4(i);
    rx(i) = str_rx;
    
    str_ry = data_table.Var5(i);
    ry(i) = str_ry;

    str_rz = data_table.Var6(i);
    rz(i) = str_rz;
end

figure
hold on
plot(str_k1, 'k')
plot(rx-rx(1), 'r')
plot(ry-ry(1), 'g')
plot(rz-rz(1), 'b')

