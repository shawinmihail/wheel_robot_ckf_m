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

%% ctrl state
% mes
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf/ctrl_state');
msgs = readMessages(selection,'DataFormat','struct');
x = cellfun(@(m) double(m.Data(1)), msgs);
y = cellfun(@(m) double(m.Data(2)), msgs);
vx = cellfun(@(m) double(m.Data(3)), msgs);
vy = cellfun(@(m) double(m.Data(4)), msgs);
yaw = cellfun(@(m) double(m.Data(5)), msgs);

%% ctrl state
% mes
selection = select(bag,'Time',[ts tf],'Topic','/wr_control/control_log');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1}
stamp_control = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
stamp_control = stamp_control - stamp_control(1);
u = cellfun(@(m) double(m.Ang), msgs);
v = cellfun(@(m) double(m.Vel), msgs);
d = cellfun(@(m) double(m.D), msgs);
psi = cellfun(@(m) double(m.Psi), msgs);
k = cellfun(@(m) double(m.K), msgs);
isStartingManeuver = cellfun(@(m) double(m.IsStartingManeuver), msgs);
goawayflag = cellfun(@(m) double(m.Goawayflag), msgs);
isAtDomain = cellfun(@(m) double(m.IsAtDomain), msgs);
cnt = cellfun(@(m) int32(m.EKFDataCnt), msgs);


%% steering
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
% plot(tyaw, yaw*180 / pi, 'b')
% plot(stamp_control, x, 'r')
% plot(stamp_control, y, 'g')
plot(stamp_control, u, 'r')
% plot(stamp_control, v, 'g')
% plot(stamp_control, d, 'c')
% plot(stamp_control, psi * 180 / pi, 'm')
% plot(stamp_control, isStartingManeuver);
% plot(stamp_control, isAtDomain,'k');
plot(i, cnt,'k');
ret

%%plot(stamp_steer_targ, angle_steer_targ, 'k')
plot(stamp_steer_mes, angle_steer_mes, 'b')

legend('ang','vel','D','\Psi','isStartMan','isAtDom', 'ang_meas');
%%
