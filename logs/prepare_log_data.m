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
