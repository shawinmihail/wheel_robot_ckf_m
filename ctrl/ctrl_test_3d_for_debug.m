clc
clear
close all


%% initial values and constants
%rand
seed = 200;
rng(seed);

% loop
dt = 1e-3;
N = 10500;

% surf
[surf_fcn, grad_surf] = plane_surf();
% [surf_fcn, grad_surf] = custom_surf();

% initial
initial_x = 9;
initial_y = -8;
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
8 8 9 10 11
];

yt = [
-10 -2 -1 -0.5 -0.5
];

zt = 0 * xt + 1.00;
set = [xt; yt; zt];

[splines] = M_spline_from_set(set);

for i = 1:N
    
    i    
    
    %% actuators dyn modeling
%     next_ctrl = [3 ; 0.1];
    y = curr_state(1:3);
    q = curr_state(7:10);
    C = quat2matrix(q);
    v = 1;
    [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, splines);
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

