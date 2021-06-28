clc
clear
close all
format long

%% read file
path = 'bag_logs/log_10_june_car2/mnt/';
path = 'bag_logs/log_10_june_car3/';
path = path + "_2021-06-10-19-51-43.bag";

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
vx = cellfun(@(m) double(m.Vel.X), msgs);
vy = cellfun(@(m) double(m.Vel.Y), msgs);
vz = cellfun(@(m) double(m.Vel.Z), msgs);
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
N = length(t_est_state);
v_ex_f = 0;
v_ey_f = 0;
v_ez_f = 0;
for i = 1:N
    q = [qw(i);qx(i);qy(i);qz(i)];
    ex = quatRotate(q, [1;0;0]);
    ey = quatRotate(q, [0;1;0]);
    ez = quatRotate(q, [0;0;1]);    spline_cfs
    v = [vx(i); vy(i); vz(i)];
    nv = norm(v);
    v_ex(i) = dot(v, ex);
    v_ey(i) = dot(v, ey);
    v_ez(i) = dot(v, ez);
    
    K = 0.10;
    v_ex_f = ekf4_smooth_K(v_ex(i), v_ex_f, K);
    v_ex_fs(i) = v_ex_f;
    
    v_ey_f = ekf4_smooth_K(v_ey(i), v_ey_f, K);
    v_ey_fs(i) = v_ey_f;
    
    v_ez_f = ekf4_smooth_K(v_ez(i), v_ez_f, K);
    v_ez_fs(i) = v_ez_f;
end

plot(t_est_state, v_ex, 'k')
plot(t_est_state, v_ex_fs, 'r')
 
% plot(t_est_state, v_ey, 'k')
% plot(t_est_state, v_ey_fs, 'g')
 
% plot(t_est_state, v_ez, 'k')
% plot(t_est_state, v_ez_fs, 'b')
 
 

