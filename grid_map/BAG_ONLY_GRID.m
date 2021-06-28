clc
clear
close all
format long

%% lidar transform
q_lid0 = [1;0;0;0];
r_lid0 = [0.7; 0.0; 0.5];


%% read file
path1 = 'grid_map/mnt/_2021-05-17-17-41-21.bag';

bag = rosbag(path1);
bag.AvailableTopics
ts1 = bag.StartTime;
tf1 = bag.EndTime;

%% grd_map
selection = select(bag,'Time',[ts1 tf1],'Topic','/wr_map/grid_map');
msgs = readMessages(selection,'DataFormat','struct');
oc_grid_sizes = cellfun(@(m) double(m.Size), msgs);
oc_grid_resolutions = cellfun(@(m) double(m.Resolution), msgs);
oc_grid_dims = cellfun(@(m) double(m.Dimention), msgs);
stamp_grid = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
map_raws = cellfun(@(m) m.Map, msgs, 'UniformOutput', 0);


%% obstcl vects
selection = select(bag,'Time',[ts1 tf1],'Topic','/wr_map/obstacles_vectors');
msgs = readMessages(selection,'DataFormat','struct');
stamp_vects= cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs);
obstcl_vects = cellfun(@(m) m.Vectors, msgs, 'UniformOutput', 0);

figure('Renderer', 'painters', 'Position', [100 100 800 600])
axis equal
hold on
%% animate without synk
n = min(length(obstcl_vects), length(map_raws));
for i = 1:n
k = mod(i,n-1) + 1
% M = 10;
% k = M;

map_raw = map_raws{k};
mdim = oc_grid_dims(1);
map_matrix = double(reshape(map_raw, [mdim, mdim]));
s = size(map_matrix);
map_matrix(1,1) = 50;
map_matrix(end,end) = -50;

msize = s(1);
mres = oc_grid_resolutions(1);

[X,Y] = meshgrid([-mres * msize/2: mres :mres * msize/2]);
plot_s = surf(X,Y,X*0, map_matrix');

vec = obstcl_vects{k};

centers_array = [];
for m = 1:length(vec)
    plot_wr = plot(vec(m).X, vec(m).Y, 'ro');
    centers_array = [centers_array plot_wr];
end
% ret
pause(0.05)
delete(plot_s);
for m = 1:length(centers_array)
    delete(centers_array(m));
end

xlim([-30 10])
ylim([-10 30])
end=