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
figure
hold on
axis equal
xlabel('X')
ylabel('Y')
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


% find cels
for i = 1:n
    p = r_obs(:,i);
    index = floor([p(1) / oc_grid_resolution + oc_grid_dim / 2; p(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    oc_grid_matrix(index(1), index(2)) = 1;
    
    free_ray = p - world_frame_lid;
    n_free_ray = norm(free_ray);
    for l = 0:oc_grid_resolution/1:(n_free_ray-oc_grid_resolution/2)
        free_point = world_frame_lid + free_ray/n_free_ray * l;
        index = floor([free_point(1) / oc_grid_resolution + oc_grid_dim / 2; free_point(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
%         if oc_grid_matrix(index(1), index(2)) == 0
            oc_grid_matrix(index(1), index(2)) = 2;
%         end
    end
end
% oc_grid_matrix
% 
figure
imagesc(oc_grid_matrix')



