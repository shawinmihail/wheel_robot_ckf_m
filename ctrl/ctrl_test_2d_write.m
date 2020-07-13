clc
clear
close all


%% initial values and constants
%rand
seed = 200;
rng(seed);

% loop
dt = 1e-2;
N = 999;

% surf
[surf_fcn, grad_surf] = custom_surf();

% initial
initial_x = 0;
initial_y = -1;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = 0/3;
initial_q = calc_q_full(grad_surf, initial_r, initial_yaw);
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
for i = 1:N
    
    i    
    
    %% actuators dyn mo deling
    next_ctrl = [3 ; 0.1];   
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


 x_points = act_states(1, 1:50:end);
 y_points = act_states(2, 1:50:end);
 z_points = act_states(3, 1:50:end);
 traj3d = [x_points; y_points; z_points];
 save('ctrl/traj3d', 'traj3d')
 
 
figure
hold on
grid on
plot3(traj3d(1, :), traj3d(2, :), traj3d(3, :),'b*')

