clc
clear
close all


%% initial values and constantspoten
%rand0
seed = 565;
% rng(seed);

% loop
dt = 1e-2;
N = 6000;

% surf
[surf_fcn, grad_surf] = plane_surf();
surf_fcn = @(x,y)(surf_fcn(x,y) - 2.5);
% [surf_fcn, grad_surf] = custom_sur && (dot1 > 0)f();

% initial
initial_x = -11.5;
initial_y = 4.1;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = 180*pi/180;
initial_q = calc_q_full(grad_surf, initial_r,initial_yaw);
initial_w = [0;0;0];
initial_state = [initial_r; initial_v; initial_q];
initial_ctrl = [0; 0];



%% preproc
curr_state = initial_state;
curr_yaw = initial_yaw;
curr_w = initial_w;
curr_ctrl = initial_ctrl;

act_states = [];
timeline = [];
%% sim
t = 0;

%  route
tb = readtable('car2.csv');
set = tb{1:50,1:end};
set = set';
[splines] = M_spline_from_set(set); 

spline_index = 1;
error_cntr = 0;

status = 1;
obstacle_list = [];

% maneur
maneur_displacement = 2;
maneur_radius = 1.0;
[virtual_set, virtual_way, virtual_zone_center, virtual_dir] = maneuver_zone_params(maneur_displacement, maneur_radius, set);
angle_eps = 10*pi/180;
in_domain = 0;
mode = 1;
mode_fixed = 0;
maneur_radius_eps = 0.1;
inited = 0;
start_u = 1;
start_v = 1;
for i = 1:N
    
    i  

    y = curr_state(1:3);
    q = curr_state(7:10);
    C = quat2matrix(q);
    u = 0;
    v = 0;
    
    in_domain = check_in_domain(q, y, virtual_dir, splines(:,:,1));
    
    status = define_status(status, q, y, virtual_zone_center, maneur_radius, maneur_radius_eps, virtual_dir, angle_eps, in_domain);
    if inited == 0
        inited = 1;
%         [start_u, start_v] = define_maneuver_start_dir(q, y, virtual_zone_center, virtual_dir);
    end
        
    if status == 2
        [u, v, mode, mode_fixed, angle] = inside_zone_dir_control(q, y, virtual_dir, virtual_way, virtual_zone_center, ...
        maneur_radius, angle_eps, maneur_radius_eps, 20*pi/180, mode, mode_fixed, start_u, start_v);
    elseif status == 3
        [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, virtual_way, obstacle_list);
        v = 1;
    elseif status == 4
        [u, v, sstar, pstar, DELTA, spline_index, error_cntr, delta_h, dot_p]  = calculate_ctrl_3d_grad_pot_avoidance(y, v, C, splines, spline_index, error_cntr, obstacle_list);
    end

%     us(:, i) = u;
%     sstars(: ,i) = sstar;
%     pstars(:, i) = pstar;
%     deltas(:, i) = C' * DELTA;
%     deltas_norm(:, i) = norm(DELTA);
%     ts(i) = t;
    
    next_ctrl = [v ; u];
 
    %% wheel robot state evolution
    % state = [r v q]
    [next_state, next_yaw] = calculate_next_state(curr_state, curr_yaw, next_ctrl, grad_surf, dt);    
    [next_a, next_w] = calculate_acc_rotrate(curr_state, next_state, dt);
    next_w_dot = (next_w - curr_w) / dt;
    
    %% full state = [r v a q w]
    full_state_curr = [next_state(1:6);next_a;next_state(7:10);next_w; next_w_dot];

    %% sim next step
    curr_ctrl = next_ctrl;
    curr_state = next_state;
    curr_yaw = next_yaw;
    curr_w = next_w;
    t = t + dt;
    
    %% save
    act_states(:, i) = full_state_curr;
    timeline(i) = t;
end

% figure
% hold on
% title('delta')
% plot(deltas_norm)


% figure
% hold on
% title('us')
% plot(timeline, us)
% 
% figure
% hold on
% title('sstars')
% plot(timeline, sstars)
% 
% figure
% hold on
% plot(timeline, deltas(1, :), 'r')
% plot(timeline, deltas(2, :), 'g')
% plot(timeline, deltas(3, :), 'b')
% 
% figure
% hold on
% fs = 20;
% set(gca,'FontSize',fs)
% title('\delta(t)')
% xlabel('t, s')
% ylabel('\delta, m')
% plot(timeline, deltas_norm, 'r')

figure
hold on
grid on
axis equal
splines_s = size(splines);
for i = 1:splines_s(3)

spline = splines(:,:,i);
point = spline * [1; 0; 0; 0];
plot3(point(1), point(2), point(3), 'g*')

for a = 0:0.1:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'r.')
end

end

% plot3(act_states(1,1),act_states(2,1),act_states(3,1), 'r*');
plot3(act_states(1,:),act_states(2,:),act_states(3,:), 'k');
% plot3(obstacle_list(:,1), obstacle_list(:,2), obstacle_list(:,3), '*','Color','g','MarkerSize',10);

splines_s = size(virtual_way);
for i = 1:splines_s(3)

spline = virtual_way(:,:,i);
point = spline * [1; 0; 0; 0];
plot3(point(1), point(2), point(3), 'k*')

for a = 0:0.1:1
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'k.')
end

end

a = 0:0.05:2*pi;
x = virtual_zone_center(1) +  (maneur_radius+maneur_radius_eps) * cos(a);
y = virtual_zone_center(2) +  (maneur_radius+maneur_radius_eps) * sin(a);
plot(x, y, 'r')

a = 0:0.05:2*pi;
x = virtual_zone_center(1) +  (maneur_radius-maneur_radius_eps) * cos(a);
y = virtual_zone_center(2) +  (maneur_radius-maneur_radius_eps) * sin(a);
plot(x, y, 'r')
