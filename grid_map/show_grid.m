clc
clear
close all
format long

%% lidar transform
q_lid0 = [1;0;0;0];
r_lid0 = [0.7; 0.0; 0.5];


%% read file
path1 = 'grid_map/_2021-05-13-12-06-00.bag';

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

% figure('Renderer', 'painters', 'Position', [250 250 2000 2000])
figure
axis equal
hold on
%% plot
res = oc_grid_resolutions(1);
sz = oc_grid_sizes(1);
dm = oc_grid_dims(1);

k = 20;
map_raw = map_raws{k};
map_matrix = reshape(map_raw, [dm, dm]);
% map_matrix = [map_matrix(2:end,:); zeros(1,dm)];
for i = 1:101
    map_matrix(1,i) = -51 + i;
end
[X,Y] = meshgrid([-sz/2:res:sz/2]);
plot_s = surf(X,Y,X*0, map_matrix');
vec = obstcl_vects{t_vects_index};
centers_array = [];
for m = 1:length(vec)
    plot_wr = plot(vec(m).X, vec(m).Y, 'ro');
end