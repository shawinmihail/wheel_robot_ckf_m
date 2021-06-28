clc
clear
close all
format long

%% read file
path1 = '_2021-02-18-21-18-33.bag';
path1 = '_2021-02-19-13-28-55.bag';


bag1 = rosbag(path1);
bag1.AvailableTopics;
ts1 = bag1.StartTime;
tf1 = bag1.EndTime;
%% 
% lidar
selection1 = select(bag1,'Time',[ts1 tf1],'Topic','/scan_filtered');
msgs1 = readMessages(selection1,'DataFormat','struct');
stamp_lid0 = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs1);
stamp_lid = stamp_lid0 - stamp_lid0(1);
Ranges = cellfun(@(m) double(m.Range), msgs1, 'UniformOutput', false); 
Theta = cellfun(@(m) double(m.Theta), msgs1, 'UniformOutput', false); 

% est_state
selection1 = select(bag1,'Time',[ts1 tf1],'Topic','/wr_ekf/ctrl_state ');
msgs1 = readMessages(selection1,'DataFormat','struct');
stamp_est0 = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs1);
stamp_est = stamp_est0 - stamp_est0(1);
xs = cellfun(@(m) double(m.X), msgs1);
ys = cellfun(@(m) double(m.Y), msgs1);
yaws = cellfun(@(m) double(m.Yaw), msgs1);
% plot(xs, ys)

% oc grid initial
oc_grid_size = 40;
oc_grid_resolution = 0.33;
% oc_grid_resolution = 1.25;
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
oc_grid_matrix = zeros(oc_grid_dim, oc_grid_dim);
for i = 1:101
    oc_grid_matrix(1,i) = -51 + i;
end

% lidar transform
q_lid = [1;0;0;0];
r_lid = [0.0; 0.0; 0.0];

% curr state
% range_i = [0;0;0];
% q = [100;0;0;0];
% q = q / norm(q);
% world_frame_lid = range_i + quatRotate(q, r_lid);

figure('Renderer', 'painters', 'Position', [250 250 2000 2000])
axis equal
hold on
for m = 1:999
k = mod(m,size(Ranges,1)-800) + 800;
% k = m;
t = stamp_lid(k);

[dv,t_index] = min(abs(t-stamp_est));
t_est = stamp_est(t_index);
yaw = yaws(t_index);
x = xs(t_index);
y = ys(t_index);
q = quatFromEul([0;0;yaw]);
wr_index = index_of_point([x;y], oc_grid_size, oc_grid_resolution);
world_frame_lid = [x; y; 0] + quatRotate(q, r_lid);

range = Ranges{k,:};
theta = (Theta{k,:});
n = length(range);

for i = 1:n
    range_i = range(i);
    lid_frame_obs = [range_i*cos(theta(i)); range_i*sin(theta(i)); 0];
    wr_frame_obs = quatRotate(q_lid, lid_frame_obs) + r_lid;
    world_frame_obs = quatRotate(q, wr_frame_obs) + world_frame_lid;
    r_obs(:,i) = world_frame_obs;
end

% find cels
oc_grid_matrix = refresh_grid_additional(r_obs*0+world_frame_lid, r_obs, oc_grid_matrix, oc_grid_size, oc_grid_resolution);
% oc_grid_matrix(wr_index(1), wr_index(2)) = 3;

% plot
[X,Y] = meshgrid([-oc_grid_size/2:oc_grid_resolution:oc_grid_size/2]);
plot_s = surf(X,Y,X*0, oc_grid_matrix');
plot_wr = plot(x,y, 'ro');
pause(0.05)
delete(plot_s);
delete(plot_wr);
end
