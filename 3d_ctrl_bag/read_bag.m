clc
clear
close all
format long

%% read file
path = '_2021-02-11-19-37-08.bag';

bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime;
tf = bag.EndTime;

%% ctrl state
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf/ctrl_state');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
x = cellfun(@(m) double(m.X), msgs);
y = cellfun(@(m) double(m.Y), msgs);
yaw = cellfun(@(m) double(m.Yaw), msgs);

%% steering
selection = select(bag,'Time',[ts tf],'Topic','/wr_sensors/steering_angle_measured');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_steer_mes = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
a_mes = cellfun(@(m) double(m.Angle.Data), msgs);
selection = select(bag,'Time',[ts tf],'Topic','/wr_sensors/steering_angle_target');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_steer_tg = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
a_tg = cellfun(@(m) double(m.Angle.Data), msgs);

%% ctrl
selection = select(bag,'Time',[ts tf],'Topic','/wr_control/control');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_ctrl = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
u = cellfun(@(m) double(m.Ang), msgs);

%% ctrl log
selection = select(bag,'Time',[ts tf],'Topic','/wr_control/control_log');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1}
t_log = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
t_log = t_log - t_log(1);
ok = cellfun(@(m) double(m.Ok), msgs);
sstar = cellfun(@(m) double(m.Sstar), msgs);
px = cellfun(@(m) double(m.P.X), msgs);
py = cellfun(@(m) double(m.P.Y), msgs);
pz = cellfun(@(m) double(m.P.Z), msgs);
dpx = cellfun(@(m) double(m.Dp.X), msgs);
dpy = cellfun(@(m) double(m.Dp.Y), msgs);
dpz = cellfun(@(m) double(m.Dp.Z), msgs);
ddpx = cellfun(@(m) double(m.Ddp.X), msgs);
ddpy = cellfun(@(m) double(m.Ddp.Y), msgs);
ddpz = cellfun(@(m) double(m.Ddp.Z), msgs);
delta = cellfun(@(m) double(m.Delta), msgs);

figure
hold on
% plot(x,y,'r')
% plot(px,py,'k')
% plot(t_steer_tg, a_tg, 'r')
% plot(t_ctrl, u, 'b')
% plot(t_log, delta*5, 'k')
plot(t_log, ok, 'r')


