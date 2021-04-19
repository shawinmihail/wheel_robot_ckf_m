clc
clear
close all


%% initial values and constants
%rand0
seed = 200;
rng(seed);

% loop
dt = 2e-3;
N = 10500;

% surf
[surf_fcn, grad_surf] = plane_surf();
% [surf_fcn, grad_surf] = custom_surf();

% initial
initial_x = 8.5;
initial_y = 1;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = pi/2;
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
%% traj
xt = [
8.21600326006528;
8.21600326006528;
8.20758457882733;
8.69140109578713;
10.1710371562285;
12.2270574729172;
14.2766380038332;
15.7387802050626;
16.1990078075244;
15.5268592055012;
13.9128696601541;
11.5072489414890
]';

yt = [
0
2.98925005022913;
5.98628231571288;
8.06028145277067;
9.59202163069941;
10.1472975092063;
9.56870396277120;
8.02025602141151;
5.94089520714774;
3.92006193799355;
2.53060571605069;
1.05548560520278
]';

% xt = [
% 8 8 8 ...
% 9 10 11 14 ...
% ];
% 
% yt = [
% -10 -6 -2 ...
% -1 -0.5 -0.5 ...
% -1
% ];

zt = 0 * xt + 1.00;
set = [xt; yt; zt];

[splines] = M_spline_from_set(set);
spline_index = 1;
error_cntr = 0;

obstacle_list = [8.05, 4.5, 1];
for i = 1:N
    
    i    
    
    %% actuators dyn modeling
%     next_ctrl = [3 ; 0.1];
    y = curr_state(1:3);
    q = curr_state(7:10);
    C = quat2matrix(q);
    v = 1;
%     [u, v, sstar, pstar, DELTA, spline_index, error_cntr]  = calculate_ctrl_3d_with_index(y, v, C, splines, spline_index, error_cntr);
    [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, splines, obstacle_list);    
    error_cntrs(i) = error_cntr;
    spline_indexs(i) = spline_index;
    us(:, i) = u;
    sstars(: ,i) = sstar;
    pstars(:, i) = pstar;
    deltas(:, i) = C' * DELTA;
    deltas_norm(:, i) = norm(DELTA);
    
    next_ctrl = [v ; u];
%     next_ctrl = process_control_input(curr_ctrl, next_ctrl, dt);
    
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

figure
hold on
title('delta')
plot(deltas_norm)


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
plot3(set(1, :), set(2, :), set(3, :),'b*')

splines_s = size(splines);
for i = 1:splines_s(3)

spline = splines(:,:,i);

for a = 0:0.1:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'r.')
end

end

plot3(act_states(1,1),act_states(2,1),act_states(3,1), 'r*');
plot3(act_states(1,:),act_states(2,:),act_states(3,:), 'k--');
plot3(obstacle_list(:,1), obstacle_list(:,2), obstacle_list(:,3), '*','Color','g','MarkerSize',10);
