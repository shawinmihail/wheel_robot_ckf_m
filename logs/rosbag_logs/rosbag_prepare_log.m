clc
clear
close all
format long

%% read file
path = 'logs/rosbag_logs/oct_14_2020/_2020-10-15-10-59-47.bag';
steering_logged = 1;
bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime + 2;
% tf = bag.EndTime;
tf = ts + 15;

%% gnns
% base fix
selection = select(bag,'Time',[ts tf],'Topic','/gnss_base/nl_triplet');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

stamp_triplet = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
% status base
s_base = cellfun(@(m) m.StatusMaster, msgs);
% r base
r_base_x = cellfun(@(m) m.REcefMaster.X, msgs);
r_base_y = cellfun(@(m) m.REcefMaster.Y, msgs);
r_base_z = cellfun(@(m) m.REcefMaster.Z, msgs);
% v base
v_base_x = cellfun(@(m) m.VEcefMaster.X, msgs);
v_base_y = cellfun(@(m) m.VEcefMaster.Y, msgs);
v_base_z = cellfun(@(m) m.VEcefMaster.Z, msgs);
% status slave 1
s_slave1 = cellfun(@(m) m.StatusSlave1, msgs);
% dr slave 1
dr_slave1_x = cellfun(@(m) m.DrEcefSlave1.X, msgs);
dr_slave1_y = cellfun(@(m) m.DrEcefSlave1.Y, msgs);
dr_slave1_z = cellfun(@(m) m.DrEcefSlave1.Z, msgs);
% status slave 2
s_slave2 = cellfun(@(m) m.StatusSlave2, msgs);
% dr slave 2
dr_slave2_x = cellfun(@(m) m.DrEcefSlave2.X, msgs);
dr_slave2_y = cellfun(@(m) m.DrEcefSlave2.Y, msgs);
dr_slave2_z = cellfun(@(m) m.DrEcefSlave2.Z, msgs);


%% imu
selection = select(bag,'Time',[ts tf],'Topic','/icm20608');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
stamp_imu = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec) / 1e9, msgs);
ax = cellfun(@(m) double(m.LinearAcceleration.X), msgs);
ay = cellfun(@(m) double(m.LinearAcceleration.Y), msgs);
az = cellfun(@(m) double(m.LinearAcceleration.Z), msgs);

wx = cellfun(@(m) double(m.AngularVelocity.X), msgs);
wy = cellfun(@(m) double(m.AngularVelocity.Y), msgs);
wz = cellfun(@(m) double(m.AngularVelocity.Z), msgs);


%% steering
if steering_logged
% mes
selection = select(bag,'Time',[ts tf],'Topic','/steering_angle_measured');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
stamp_steer_mes = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
angle_steer_mes = cellfun(@(m) double(m.Angle.Data), msgs);
% targ
selection = select(bag,'Time',[ts tf],'Topic','/steering_angle_target');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};
stamp_steer_targ = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
angle_steer_targ = cellfun(@(m) double(m.Angle.Data), msgs);
end

%% time
ts_triplet = stamp_triplet-stamp_triplet(1);
ts_imu = stamp_imu-stamp_triplet(1);
% ts_imu = ts_imu(ts_imu > 0);
if steering_logged
ts_steering = stamp_steer_mes - stamp_triplet(1);
% ts_steering = ts_steering(ts_imu > 0);
end

%% groupping
array_triplet = ...
[r_base_x, r_base_y, r_base_z,...
v_base_x, v_base_y, v_base_z, ...
dr_slave1_x, dr_slave1_y, dr_slave1_z, ...
dr_slave2_x, dr_slave2_y, dr_slave2_z, ...
double(s_base), double(s_slave1), double(s_slave2)];
array_imu = [ax ay az wx wy wz];
if steering_logged
array_steering = angle_steer_mes;
end


%% save
save('logs/ts_triplet', 'ts_triplet')
save('logs/array_triplet', 'array_triplet')
save('logs/ts_imu', 'ts_imu')
save('logs/array_imu', 'array_imu')
if steering_logged
save('logs/ts_steering', 'ts_steering')
save('logs/array_steering', 'array_steering')
end


%% calib slaves
calib_duration = 10; %s
t_step = 1e-3;


calib_r_buffer = [];
calib_v_buffer = [];
calib_dr1_buffer = [];
calib_dr2_buffer = [];
calib_v_max = 0.05;
calib_v_rms_max = 0.05;
ts_slaves_calib = 0;
tf_slaves_calib = 0;

calib_procedure_runs = 0;
calib_procedure_finished = 0;
slave1_dr = 0;
slave2_dr = 0;
i = 0;
for t = 0:t_step:ts_triplet(end)
    
i = i + 1;
[is_event, stamp, measure] = measured_event(t, t-t_step, ts_triplet, array_triplet);


if is_event
    [calib_procedure_finished, calib_procedure_runs, dr1, dr2,...
    calib_r_buffer, calib_v_buffer, calib_dr1_buffer, calib_dr2_buffer, ts_slaves_calib, tf_slaves_calib] = ...
    ...
    slaves_calib(t, measure, ...
    calib_r_buffer, calib_v_buffer, calib_dr1_buffer, calib_dr2_buffer, ...
    calib_procedure_finished, calib_procedure_runs, ts_slaves_calib, tf_slaves_calib,...
    calib_duration, calib_v_max, calib_v_rms_max);

    if calib_procedure_finished
        slave1_dr = dr1;
        slave2_dr = dr2;
        break
    end
end

    
    
end

save('logs/slave1_dr', 'slave1_dr')
save('logs/slave2_dr', 'slave2_dr')
[lat_ref, lon_ref, alt_ref] = ecef2wgs(r_base_x(1), r_base_y(2), r_base_z(3));
wgs_ref = [lat_ref lon_ref alt_ref];
save('logs/wgs_ref', 'wgs_ref')

ret


N = length(r_base_x);
for i = 1:N
    vn(i) = norm([v_base_x(i); v_base_y(i); v_base_z(i)]);
end
figure
hold on
grid on
plot(stamp_triplet-stamp_triplet(1), vn)
plot(stamp_triplet-stamp_triplet(1), v_base_x)
plot(stamp_triplet-stamp_triplet(1), v_base_y)
plot(stamp_triplet-stamp_triplet(1), v_base_z)

%%
N = length(r_base_x);
x0 = r_base_x(1);
y0 = r_base_y(1);
z0 = r_base_z(1);
[phi0, lambda0, h0] = ecef2wgs(x0, y0, z0);
R_ecef_enu = rmx2enu(phi0, lambda0);
R = R_ecef_enu;
for i = 1:2
    x = r_base_x(i);
    y = r_base_y(i);
    z = r_base_z(i);
    dx1 = dr_slave1_x(i);
    dy1 = dr_slave1_y(i);
    dz1 = dr_slave1_z(i);
    dx2 = dr_slave2_x(i);
    dy2 = dr_slave2_y(i);
    dz2 = dr_slave2_z(i);

    r_base_enu(:,i) = R*[(x-x0);(y-y0);(z-z0)];
    r_slave1_enu(:,i) = R*[(x+dx1-x0);(y+dy1-y0);(z+dz1-z0)];
    r_slave2_enu(:,i) = R*[(x+dx2-x0);(y+dy2-y0);(z+dz2-z0)];
end

figure
hold on
grid on

plot(angle_steer_mes, 'r')
plot(angle_steer_targ, 'k--')

ret

% plot(r_slave1_enu(1, :), 'r')
% plot(r_slave1_enu(2, :), 'g')
% plot(r_slave1_enu(3, :), 'b')

% plot(r_slave2_enu(1, :), 'r')
% plot(r_slave2_enu(2, :), 'g')
% plot(r_slave2_enu(3, :), 'b')
% 
% plot(r_base_enu(1, :), 'r--')
% plot(r_base_enu(2, :), 'g--')
% plot(r_base_enu(3, :), 'b--')


