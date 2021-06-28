clc
clear
close all
format long

%% read file
path = 'bag_logs/log_16_june_car2/';
% path = 'bag_logs/log_16_june_car3/';
path = path + "_2021-06-16-19-50-02.bag";

bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime;
tf = bag.EndTime;

%% est state
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf/est_state');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
t_est_state = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
x = cellfun(@(m) double(m.Pos.X), msgs);
y = cellfun(@(m) double(m.Pos.Y), msgs);
z = cellfun(@(m) double(m.Pos.Z), msgs);
qw = cellfun(@(m) double(m.Att.W), msgs);
qx = cellfun(@(m) double(m.Att.X), msgs);
qy = cellfun(@(m) double(m.Att.Y), msgs);
qz = cellfun(@(m) double(m.Att.Z), msgs);


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
msgs{1};
t_log = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
ok = cellfun(@(m) double(m.Ok), msgs);
index = cellfun(@(m) double(m.CfsIndex), msgs);
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
delta = cellfun(@(m) double(m.DeltaNorm), msgs);
DeltaX = cellfun(@(m) double(m.DeltaVect.X), msgs);
DeltaY = cellfun(@(m) double(m.DeltaVect.Y), msgs);
DeltaZ = cellfun(@(m) double(m.DeltaVect.Z), msgs);

figure
hold on
grid on
plot(t_steer_tg, a_tg, 'r')
% plot(t_steer_mes, a_mes, 'k')
plot(t_log, delta*10, 'k')
% plot(t_est_state, y, 'g')

figure
hold on
grid on
plot(a_tg, 'r')
plot(a_mes, 'k')


%%
figure
hold on
grid on
axis equal

% N = length(x);
% for n = 1:50:N%\numberthis
% qn = [qw(n); qx(n); qy(n); qz(n)];
% rn = [x(n); y(n); z(n)];
% plot_frame(rn, qn)
% end

plot3(x,y,z, 'k')

plot3(px, py, pz, 'g.')

tb = readtable('logs/routes/path_june1_along_car1.csv');
set = tb{:,:};
set = set';

[splines] = M_spline_from_set(set);

splines_length = length(splines);
for i = 1:splines_length

spline = splines(:,:,i);
point = spline * [1; 0; 0; 0];
% text(point(1), point(2), num2str(i-1))

for a = 0:0.33:1
    spline;
    point = spline * [1; a; a^2; a^3];
%     plot3(point(1), point(2), point(3), 'r.')
end

end

