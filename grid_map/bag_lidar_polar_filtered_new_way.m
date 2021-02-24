clc
clear
close all
format long

%% read file
path1 = '_2021-02-18-21-18-33.bag';


bag1 = rosbag(path1);
% bag1.AvailableTopics
ts1 = bag1.StartTime;
tf1 = bag1.EndTime;
%% 
% mes
selection1 = select(bag1,'Time',[ts1 tf1],'Topic','/scan_filtered');
msgs1 = readMessages(selection1,'DataFormat','struct');
stamp0 = cellfun(@(m) double(m.Stamp.Sec) + double(m.Stamp.Nsec) / 1e9, msgs1);
stamp = stamp0 - stamp0(1);
Ranges = cellfun(@(m) double(m.Range), msgs1, 'UniformOutput', false); 
Theta = cellfun(@(m) double(m.Theta), msgs1, 'UniformOutput', false);   
%%
speed = 16;

% for i = 1:size(Ranges,1)
% % obs_theta = Theta{i,:}*pi/180;
% obs_theta = (Theta{i,:});
% obs_r = Ranges{i,:};
% 
% % rej = 2*pi/3;
% % rej_ = pi/60;
% % obs_r = obs_r((obs_theta<(pi-rej))|(obs_theta>(pi+rej+rej_)));
% % obs_theta = obs_theta((obs_theta<(pi-rej))|(obs_theta>(pi+rej+rej_)));
% 
% h = polarplot(obs_theta, obs_r,'.', 'LineWidth', 2);
% rlim([0 5])
% 
% if (i == 1)
%     pause(stamp(i)/speed);
% else
%     pause((stamp(i)-stamp(i-1))/speed);
% end
% end
% ret

% oc grid initial
oc_grid_size = 30;
oc_grid_resolution = 0.35;
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
oc_grid_matrix = zeros(oc_grid_dim, oc_grid_dim);

% lidar transform
q_lid = [1;0;0;0];
r_lid = [0.0; 0.0; 0.0];

% curr state
r = [0;0;0];
q = [100;0;0;0];
q = q / norm(q);
world_frame_lid = r + quatRotate(q, r_lid);


figure
hold on
for m = 1:999
k = mod(m,size(Ranges,1)-150) + 150
range = Ranges{k,:};
theta = (Theta{k,:});
n = length(range);

% figure
% hold on
for i = 1:n
    lid_frame_obs = [range(i)*cos(theta(i)); range(i)*sin(theta(i)); 0];
    wr_frame_obs = quatRotate(q_lid, lid_frame_obs) + r_lid;
    world_frame_obs = quatRotate(q, wr_frame_obs) + world_frame_lid;
    r_obs(:,i) = world_frame_obs;
%     plot3(world_frame_obs(1),world_frame_obs(2),world_frame_obs(3), 'k.');
    
%     free_ray = world_frame_obs - r;
%     n_free_ray = norm(free_ray);
%     for l = 0:oc_grid_resolution/1:n_free_ray
%         free_point = world_frame_lid + free_ray/n_free_ray * l;
%         plot3(free_point(1),free_point(2),free_point(3), 'g.');
%     end
end

% find cels
oc_grid_matrix = refresh_grid(r_obs*0+world_frame_lid, r_obs, oc_grid_matrix, oc_grid_size, oc_grid_resolution);
k
h = imagesc(oc_grid_matrix');
pause(0.1)
clf

end

