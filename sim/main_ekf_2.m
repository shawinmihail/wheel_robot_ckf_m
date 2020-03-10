clc
clear
close all


%% initial values and constants
%rand
seed = 200;
rng(seed);

% loop
dt = 1e-3;
N = 90000;

% surf
[surf_fcn, grad_surf] = custom_surf();

% initial
initial_x = 0;
initial_y = -30;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = pi/3;
initial_q = calc_q_full(grad_surf, initial_r, initial_yaw);
initial_w = [0;0;0];
initial_state = [initial_r; initial_v; initial_q];
initial_ctrl = [0; 0];

%% scripts
run('sensors_init')
run('init_ekf2')

%% preproc
curr_state = initial_state; 
curr_yaw = initial_yaw;
curr_w = initial_w;
curr_ctrl = initial_ctrl;
est_state_curr = initial_est_state;
sqrtP_curr = initial_sqrtP;

%% sim
t = 0;
for i = 1:N
    
    i    
    
    %% actuators dyn modeling
    next_ctrl = [5 ; 0.01 - 0.001*t];   
    if t < 10
        next_ctrl = [0 ; 0];
    end
    if t > 50 && t < 70
        next_ctrl = [0 ; 0];
    end

    next_ctrl = process_control_input(curr_ctrl, next_ctrl, dt);
    
    %% wheel robot state evolution
    % state = [r v q]
    [next_state, next_yaw] = calculate_next_state(curr_state, curr_yaw, next_ctrl, grad_surf, dt);    
    [next_a, next_w] = calculate_acc_rotrate(curr_state, next_state, dt);
    next_w_dot = (next_w - curr_w) / dt;
    
    %% full state = [r v a q w]
    full_state_curr = [next_state(1:6);next_a;next_state(7:10);next_w; next_w_dot];

    %% mes_state
    mes_state_curr = mes_state_from_full_state(...
        full_state_curr, gps_pos_local_rsm, gps_vel_local_rsm, gps_quat_rsm, imu_acc_rsm, imu_rot_vel_rsm, imu_attachment_r, gps_attachment_r);
     
    %% estimation, X = [r v q]
    a_mes = mes_state_curr(7:9);
    w_mes = mes_state_curr(14:16);
    
    %% predict with imu
    [est_state_next, sqrtP_next] = ekf2_wr_prediction_imu(est_state_curr, sqrtP_curr, Q, a_mes, w_mes, dt);
    
    %% correct pos gnns
    Z = mes_state_curr(1:3);
    [est_state_next, sqrtP_next] = ekf2_wr_correction_p_gnns(est_state_next, sqrtP_next, Z, sqrtR_p_gnns, gps_attachment_r);
    Z = mes_state_curr(4:6);
    [est_state_next, sqrtP_next] = ekf2_wr_correction_u_gnns(est_state_next, sqrtP_next, Z, sqrtR_u_gnns);
    [est_state_next, sqrtP_next] = ekf2_wr_correction_v_gnns(est_state_next, sqrtP_next, Z, sqrtR_v_gnns);
    Z = mes_state_curr(7:9);
    [est_state_next, sqrtP_next] = ekf2_wr_correction_a_imu(est_state_next, sqrtP_next, Z, sqrtR_a_imu);
 
    
    %% sim next step
    curr_ctrl = next_ctrl;
    curr_state = next_state;
    curr_yaw = next_yaw;
    curr_w = next_w;
    est_state_curr = est_state_next;
    sqrtP_curr = sqrtP_next;
    t = t + dt;
    
    %% save
    est_states(:, i) = est_state_curr;
    act_states(:, i) = full_state_curr;
    mes_states(:, i) = mes_state_curr;
    sqrtP_diag(:,i) = diag(sqrtP_next);
    timeline(i) = t;
    

end





