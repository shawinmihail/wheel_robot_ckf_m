clc
clear
close all
format long

%% read file
path = 'logs/kinmodel_logs/bag1.bag';

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

%% plot
figure
hold on
grid on
title('plot')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_r_base_x-m_r_base_x(1), 'ro')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_r_base_x-m_r_base_x(1), 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_r_base_y-m_r_base_y(1), 'go')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_r_base_y-m_r_base_y(1), 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_r_base_z-m_r_base_z(1), 'bo')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_r_base_z-m_r_base_z(1), 'ko')

% plot(e_stamp_triplet-e_stamp_triplet(1),e_v_base_x-m_v_base_x(1), 'ro')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_v_base_x-m_v_base_x(1), 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_v_base_y-m_v_base_y(1), 'go')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_v_base_y-m_v_base_y(1), 'ko')
% plot(e_stamp_triplet-e_stamp_triplet(1),e_v_base_z-m_v_base_z(1), 'bo')
% plot(m_stamp_triplet-m_stamp_triplet(1),m_v_base_z-m_v_base_z(1), 'ko')


plot(e_stamp_triplet-e_stamp_triplet(1), e_dr_slave2_x, 'ro')
plot(m_stamp_triplet-m_stamp_triplet(1), m_dr_slave2_x, 'ko')
plot(e_stamp_triplet-e_stamp_triplet(1), e_dr_slave2_y, 'go')
plot(m_stamp_triplet-m_stamp_triplet(1), m_dr_slave2_y, 'ko')
plot(e_stamp_triplet-e_stamp_triplet(1), e_dr_slave2_z, 'bo')
plot(m_stamp_triplet-m_stamp_triplet(1), m_dr_slave2_z, 'ko')


