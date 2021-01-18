clc
clear
close all
format long

%% read file
path = 'logs/kin_model_logs/c1.bag';

bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime;
tf = bag.EndTime;

%% steer
% mes
selection = select(bag,'Time',[ts tf],'Topic','/wr_sensors/steering_angle_measured');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
stamp_steer_mes = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
angle_steer_mes = cellfun(@(m) double(m.Angle.Data), msgs);
t0_steer = stamp_steer_mes(1);
stamp_steer_mes = stamp_steer_mes - t0_steer;
% targ
selection = select(bag,'Time',[ts tf],'Topic','wr_sensors/steering_angle_target');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
stamp_steer_targ = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
angle_steer_targ = cellfun(@(m) double(m.Angle.Data), msgs);
stamp_steer_targ = stamp_steer_targ - t0_steer;

%% plot
figure
hold on
grid on
% title('plot')
% data = abs(diff(angle_steer_mes))
% data = data(abs(data) > 0.1 & abs(data) < 0.5)
% hist(data, 1000)
% xlim([0,0.1])
% ret
% plot(diff(stamp_steer_mes))
% ylim([0,0.01])
plot(stamp_steer_targ, angle_steer_targ, 'k')
plot(stamp_steer_mes, angle_steer_mes, 'r.')
