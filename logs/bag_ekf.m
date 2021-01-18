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

%% /wr_ekf_ros/test/est_triplet
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf_ros/test/est_triplet');
msgs = readMessages(selection,'DataFormat','struct');
e_stamp_triplet = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
e_s_base = cellfun(@(m) m.StatusMaster, msgs);
e_r_base_x = cellfun(@(m) m.REcefMaster.X, msgs);
e_r_base_y = cellfun(@(m) m.REcefMaster.Y, msgs);
e_r_base_z = cellfun(@(m) m.REcefMaster.Z, msgs);
e_v_base_x = cellfun(@(m) m.VEcefMaster.X, msgs);
e_v_base_y = cellfun(@(m) m.VEcefMaster.Y, msgs);
e_v_base_z = cellfun(@(m) m.VEcefMaster.Z, msgs);
e_s_slave1 = cellfun(@(m) m.StatusSlave1, msgs);
e_dr_slave1_x = cellfun(@(m) m.DrEcefSlave1.X, msgs);
e_dr_slave1_y = cellfun(@(m) m.DrEcefSlave1.Y, msgs);
e_dr_slave1_z = cellfun(@(m) m.DrEcefSlave1.Z, msgs);
e_s_slave2 = cellfun(@(m) m.StatusSlave2, msgs);
e_dr_slave2_x = cellfun(@(m) m.DrEcefSlave2.X, msgs);
e_dr_slave2_y = cellfun(@(m) m.DrEcefSlave2.Y, msgs);
e_dr_slave2_z = cellfun(@(m) m.DrEcefSlave2.Z, msgs);


%% /wr_ekf_ros/test/mes_triplet
selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf_ros/test/mes_triplet');
msgs = readMessages(selection,'DataFormat','struct');
m_stamp_triplet = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
m_s_base = cellfun(@(m) m.StatusMaster, msgs);
m_r_base_x = cellfun(@(m) m.REcefMaster.X, msgs);
m_r_base_y = cellfun(@(m) m.REcefMaster.Y, msgs);
m_r_base_z = cellfun(@(m) m.REcefMaster.Z, msgs);
m_v_base_x = cellfun(@(m) m.VEcefMaster.X, msgs);
m_v_base_y = cellfun(@(m) m.VEcefMaster.Y, msgs);
m_v_base_z = cellfun(@(m) m.VEcefMaster.Z, msgs);
m_s_slave1 = cellfun(@(m) m.StatusSlave1, msgs);
m_dr_slave1_x = cellfun(@(m) m.DrEcefSlave1.X, msgs);
m_dr_slave1_y = cellfun(@(m) m.DrEcefSlave1.Y, msgs);
m_dr_slave1_z = cellfun(@(m) m.DrEcefSlave1.Z, msgs);
m_s_slave2 = cellfun(@(m) m.StatusSlave2, msgs);
m_dr_slave2_x = cellfun(@(m) m.DrEcefSlave2.X, msgs);
m_dr_slave2_y = cellfun(@(m) m.DrEcefSlave2.Y, msgs);
m_dr_slave2_z = cellfun(@(m) m.DrEcefSlave2.Z, msgs);

ws = [8.21600326006528,0;
8.21600326006528,2.98925005022913;
8.20758457882733,5.98628231571288;
8.69140109578713,8.06028145277067;
10.1710371562285,9.59202163069941;
12.2270574729172,10.1472975092063;
14.2766380038332,9.56870396277120;
15.7387802050626,8.02025602141151;
16.1990078075244,5.94089520714774;
15.5268592055012,3.92006193799355;
13.9128696601541,2.53060571605069;
11.5072489414890,1.05548560520278];



figure
hold on
grid on
title('plot')

% plot(m_r_base_x,m_r_base_y, 'k')
% plot(e_r_base_x,e_r_base_y, 'r')
% plot(ws(:,1),ws(:,2), 'go')
% 
% daspect([1,1,1]);
% ret

% plot(e_stamp_triplet-e_stamp_triplet(1),e_r_base_x-m_r_base_x(1), 'r')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_r_base_x-m_r_base_x(1), 'k')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_r_base_y-m_r_base_y(1), 'g')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_r_base_y-m_r_base_y(1), 'k')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_r_base_z-m_r_base_z(1), 'b')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_r_base_z-m_r_base_z(1), 'k')

% plot(e_stamp_triplet-e_stamp_triplet(1),e_v_base_x, 'r')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_v_base_x, 'k')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_v_base_y, 'go')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_v_base_y, 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_v_base_z, 'bo')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_v_base_z, 'ko')


% plot(e_stamp_triplet-e_stamp_triplet(1), e_dr_slave2_x, 'ro')
% plot(m_stamp_triplet-m_stamp_triplet(1), m_dr_slave2_x, 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1), e_dr_slave2_y, 'go')
% plot(m_stamp_triplet-m_stamp_triplet(1), m_dr_slave2_y, 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1), e_dr_slave2_z, 'go')
% plot(m_stamp_triplet-m_stamp_triplet(1), m_dr_slave2_z, 'bo')

%%
% selection = select(bag,'Time',[ts tf],'Topic','/wr_ekf/ctrl_state');
% msgs = readMessages(selection,'DataFormat','struct');
% x = cellfun(@(m) double(m.Data(1)), msgs);
% y = cellfun(@(m) double(m.Data(2)), msgs);
% vx = cellfun(@(m) double(m.Data(3)), msgs);
% vy = cellfun(@(m) double(m.Data(4)), msgs);
% yaw = cellfun(@(m) double(m.Data(5)), msgs);
% ts = 0;
% tf = e_stamp_triplet-m_stamp_triplet(1);
% dt = tf(end)/length(yaw);
% tyaw =ts:dt:tf(end);
% tyaw = tyaw(1:end-1);
% 
figure
hold on
grid on
e_stamp_triplet = e_stamp_triplet-m_stamp_triplet(1);
m_stamp_triplet = m_stamp_triplet-m_stamp_triplet(1);
% 
% plot(diff(m_stamp_triplet))


for i = 1:length(m_dr_slave2_x)
    r1 = [m_dr_slave1_x(i),m_dr_slave1_y(i),m_dr_slave1_z(i)]';
    r2 = [m_dr_slave2_x(i),m_dr_slave2_y(i),m_dr_slave2_z(i)]';
    rhc = (r1 + r2) / 2;
    rhc_n = rhc / norm(rhc);
    ex = [1;0;0];
    cosA = dot(ex ,rhc_n);
    m_yaw(i) = acos(cosA);
%     plot(m_stamp_triplet(i), a, 'ko')
end
for i = 1:length(e_dr_slave2_x)
    r1 = [e_dr_slave1_x(i),e_dr_slave1_y(i),e_dr_slave1_z(i)]';
    r2 = [e_dr_slave2_x(i),e_dr_slave2_y(i),e_dr_slave2_z(i)]';
    rhc = (r1 + r2) / 2;
    rhc_n = rhc / norm(rhc);
    ex = [1;0;0];
    cosA = dot(ex ,rhc_n);
    e_yaw(i) = acos(cosA);
%     plot(e_stamp_triplet(i), a, 'ro')
end
plot(e_stamp_triplet, e_yaw*180/pi, 'r')
plot(m_stamp_triplet, m_yaw*180/pi, 'k')
% plot(tyaw, yaw,'b')