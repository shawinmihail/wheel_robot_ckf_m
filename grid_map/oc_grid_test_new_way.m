clc
clear
close all
rng(200)

% oc grid initial
oc_grid_size = 30;
oc_grid_resolution = 0.5;
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
oc_grid_matrix = zeros(oc_grid_dim, oc_grid_dim);

% lidar transform
q_lid = [100;0;0;100];
q_lid = q_lid/norm(q_lid);
r_lid = [2.0; 0.0; 0.0];

% curr state
r = [0;0;0];
q = [100;0;0;100];
q = q / norm(q);

% curr mes
lid_angle_lim = pi;
lid_angle_step = pi/180;
n = 0;
for angle = -lid_angle_lim/2:lid_angle_step:lid_angle_lim/2
    n = n + 1;
    theta(n) = angle;
    range(n) = 5 + 0.3 * (rand()-0.5) + angle;
end
polarplot(theta, range, '.', 'LineWidth', 2);

% mes transformation
world_frame_lid = r + quatRotate(q, r_lid);

for i = 1:n
    lid_frame_obs = [range(i)*cos(theta(i)); range(i)*sin(theta(i)); 0];
    wr_frame_obs = quatRotate(q_lid, lid_frame_obs) + r_lid;
    world_frame_obs = quatRotate(q, wr_frame_obs) + r;
    r_obs(:,i) = world_frame_obs;
    plot3(world_frame_obs(1),world_frame_obs(2),world_frame_obs(3), 'k.');
    
    free_ray = world_frame_obs - world_frame_lid;
    n_free_ray = norm(free_ray);
    for l = 0:oc_grid_resolution/1:(n_free_ray-oc_grid_resolution/2)
        free_point = world_frame_lid + free_ray/n_free_ray * l;
        plot3(free_point(1),free_point(2),free_point(3), 'g.');
    end
end


% % free
% for k = 1:n
%     p1 = world_frame_lid;
%     p2 = r_obs(:,k);
%     % begin ray opened
%     indexes = beginray_cells(p1, oc_grid_size, oc_grid_resolution);
%     s = size(indexes);
%     for i = 1:s(2)
%         index = indexes(:,i);
%         oc_grid_matrix(index(1), index(2)) = 1;
%     end
% 
%     % on ray opened
%     [indexes, xmin_d, xmax_d, ymin_d, ymax_d, x_inters_points, y_inters_points, xrays, yrays]  = onray_free_cells(p1, p2, oc_grid_size, oc_grid_resolution);
%     s = size(indexes);
%     for i = 1:s(2)
%         index = indexes(:,i);
%         oc_grid_matrix(index(1), index(2)) = 1;
%     end    
% end
% 
% % close
% for k = 1:n
%     p1 = world_frame_lid;
%     p2 = r_obs(:,k);
% 
%     % end ray closed
%     indexes = endray_cells(p2, oc_grid_size, oc_grid_resolution);
%     s = size(indexes);
%     for i = 1:s(2)
%         index = indexes(:,i);
%         oc_grid_matrix(index(1), index(2)) = 2;
%     end
% end

oc_grid_matrix = refresh_grid(r_obs*0+world_frame_lid, r_obs, oc_grid_matrix, oc_grid_size, oc_grid_resolution);

figure
hold on
axis equal
xlabel('X')
ylabel('Y')
[X,Y] = meshgrid([-oc_grid_size/2:oc_grid_resolution:oc_grid_size/2]);
surf(X,Y,X*0, oc_grid_matrix')
plot(world_frame_lid(1), world_frame_lid(2), 'ro')




