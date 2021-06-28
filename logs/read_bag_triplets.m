clc
clear
close all
format long

%% read file
path = 'bag_logs/log_10_june_car2/mnt/';
path = path + "_2021-06-10-19-52-54.bag";

bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime;
tf = bag.EndTime;

%% /wr_sensors/nl_triplet
selection = select(bag,'Time',[ts tf],'Topic','/wr_sensors/nl_triplet');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1}
ret
t_nl_tr = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
x_nl_s1 = cellfun(@(m) double(m.DrEcefSlave1.X), msgs);
y_nl_s1 = cellfun(@(m) double(m.DrEcefSlave1.Y), msgs);
z_nl_s1 = cellfun(@(m) double(m.DrEcefSlave1.Z), msgs);
x_nl_s2 = cellfun(@(m) double(m.DrEcefSlave2.X), msgs);
y_nl_s2 = cellfun(@(m) double(m.DrEcefSlave2.Y), msgs);
z_nl_s2 = cellfun(@(m) double(m.DrEcefSlave2.Z), msgs);

%% /wr_ekf_ros/test/est_triplet
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf_ros/test/est_triplet');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_ts_tr = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
x_ts_s1 = cellfun(@(m) double(m.DrEcefSlave1.X), msgs);
y_ts_s1 = cellfun(@(m) double(m.DrEcefSlave1.Y), msgs);
z_ts_s1 = cellfun(@(m) double(m.DrEcefSlave1.Z), msgs);
x_ts_s2 = cellfun(@(m) double(m.DrEcefSlave2.X), msgs);
y_ts_s2 = cellfun(@(m) double(m.DrEcefSlave2.Y), msgs);
z_ts_s2 = cellfun(@(m) double(m.DrEcefSlave2.Z), msgs);

figure
hold on
plot(z_ts_s1, 'r')
plot(z_ts_s2, 'g')
% plot(z_nl_s1, 'b')