clc
clear
close all
format long

%% read file
path = 'logs/wr_logs/wr_circles.bag';

bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime;
tf = bag.EndTime;

%%  /gnss_base/nl_triplet
selection = select(bag,'Time',[ts tf],'Topic','/gnss_base/nl_triplet');
msgs = readMessages(selection,'DataFormat','struct');
stamp_triplet = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
s_base = cellfun(@(m) m.StatusMaster, msgs);
r_base_x = cellfun(@(m) m.REcefMaster.X, msgs);
r_base_y = cellfun(@(m) m.REcefMaster.Y, msgs);
r_base_z = cellfun(@(m) m.REcefMaster.Z, msgs);
v_base_x = cellfun(@(m) m.VEcefMaster.X, msgs);
v_base_y = cellfun(@(m) m.VEcefMaster.Y, msgs);
v_base_z = cellfun(@(m) m.VEcefMaster.Z, msgs);
s_slave1 = cellfun(@(m) m.StatusSlave1, msgs);
dr_slave1_x = cellfun(@(m) m.DrEcefSlave1.X, msgs);
dr_slave1_y = cellfun(@(m) m.DrEcefSlave1.Y, msgs);
dr_slave1_z = cellfun(@(m) m.DrEcefSlave1.Z, msgs);
s_slave2 = cellfun(@(m) m.StatusSlave2, msgs);
dr_slave2_x = cellfun(@(m) m.DrEcefSlave2.X, msgs);
dr_slave2_y = cellfun(@(m) m.DrEcefSlave2.Y, msgs);
dr_slave2_z = cellfun(@(m) m.DrEcefSlave2.Z, msgs);



%% plot
figure
hold on
grid on
title('plot')
plot(r_base_x, 'ko')


