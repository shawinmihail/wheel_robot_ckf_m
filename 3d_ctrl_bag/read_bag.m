clc
clear
close all
format long

%% read file
path = '_2021-02-11-19-37-08.bag';

bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime+10;
tf = bag.EndTime-20;

%% ctrl state
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf/ctrl_state');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_ctrl_state = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
t_ctrl_state = t_ctrl_state - t_ctrl_state(1);
x = cellfun(@(m) double(m.X), msgs);
y = cellfun(@(m) double(m.Y), msgs);
yaw = cellfun(@(m) double(m.Yaw), msgs);

% %% est state
% selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf/est_state');
% msgs = readMessages(selection,'DataFormat','struct');
% msgs{1};
% t_ctrl_state = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
% t_ctrl_state = t_ctrl_state - t_ctrl_state(1);
% x = cellfun(@(m) double(m.Pos.X), msgs);
% y = cellfun(@(m) double(m.Pos.Y), msgs);
% z = cellfun(@(m) double(m.Pos.Z), msgs);


%% steering
selection = select(bag,'Time',[ts tf],'Topic','/wr_sensors/steering_angle_measured');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_steer_mes = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
t_steer_mes = t_steer_mes - t_steer_mes(1);
a_mes = cellfun(@(m) double(m.Angle.Data), msgs);
selection = select(bag,'Time',[ts tf],'Topic','/wr_sensors/steering_angle_target');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_steer_tg = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
t_steer_tg = t_steer_tg - t_steer_tg(1);
a_tg = cellfun(@(m) double(m.Angle.Data), msgs);

%% ctrl
selection = select(bag,'Time',[ts tf],'Topic','/wr_control/control');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_ctrl = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
t_ctrl = t_ctrl - t_ctrl(1);
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
plot(t_steer_tg, a_tg, 'k')
plot(t_steer_mes, a_mes, 'k--')
plot(t_ctrl, u, 'b')
% plot(t_log, ok, 'r')
% plot(t_log, delta, 'r')


figure
hold on
% plot(t_steer_tg, a_tg, 'k')
% plot(t_steer_mes, a_mes, 'k--')
% plot(t_ctrl, u, 'b')
% % plot(t_log, delta*5, 'k')
% plot(t_log, ok, 'r')
plot(t_ctrl_state, y, 'r')

%% traj
figure
hold on
grid on
plot(x,y,'r')

xt = [
8 8 9 10 11
];
yt = [
-10 -2 -1 -0.5 -0.5
];
zt = 0 * xt - 2.8;
set = [xt; yt; zt];
[splines] = M_spline_from_set(set);
plot3(set(1, :), set(2, :), set(3, :),'b*')

splines_s = size(splines);
for i = 1:splines_s(3)

spline = splines(:,:,i);

for a = 0:0.1:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'r.')
end

end




