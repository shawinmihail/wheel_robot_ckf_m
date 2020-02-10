clc
clear
close all


%% initial values and constants
seed = 200;
rng(seed);

dt = 1e-3;
N = 10000;
initial_state = [0 0 0]';
initial_state_dot = [0 0 0]';
initial_ctrl = [0 0]';
initial_est_state = [0;0;0;0;0;0;1;0;0;0];

gps_pos_local_rsm = 0.12;
gps_vel_local_rsm = 0.05;
R = diag([gps_pos_local_rsm*[1; 1; 1]; gps_vel_local_rsm*[1; 1; 1]]).^2;
Q = diag([1e-4*[1; 1; 1];    1e-4*[1; 1; 1];    1e-4*[1; 1; 1; 1]]);
P0 = 12*Q;

R_att = diag(1e-6*[1 1 1 1]);

sqrtR = chol(R,'lower');
sqrtR_att = chol(R_att,'lower');
sqrtQ = chol(Q,'lower');
initial_sqrtP = chol(P0,'lower');

%% preproc
curr_state = initial_state;
curr_state_dot = initial_state_dot;
curr_ctrl = initial_ctrl;
est_state_curr = initial_est_state;
sqrtP_curr = initial_sqrtP;

for i = 1:N
    
    i
    
    %% wheel robot state evolution state = [x y phi]
    next_ctrl = [1,0.2];
    [next_state, next_state_dot, next_ctrl] = ... 
        integrate_wheel_robot_model(curr_state, curr_state_dot, curr_ctrl, next_ctrl, dt);
    
    %% full state = [r v a q w]
    full_state_curr = full_state_from_state(next_state, next_state_dot, curr_state_dot, dt);
    
    %% mes_state
    mes_state_curr = mes_state_from_full_state(full_state_curr);
    
    %% estimation, X = [r v q]
    X = [full_state_curr(1:6); full_state_curr(10:13)];
    a_mes = mes_state_curr(7:9);
    w_mes = mes_state_curr(14:16);
    % predict
    [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_curr, sqrtP_curr, sqrtQ, a_mes, w_mes, dt);
    

    
%     correct pos vel
%     if (mod(i, 50) == 8)
%         if rand() > 0.1
            Z = mes_state_curr(1:6);
            [est_state_next, sqrtP_next] = ekf_wr_correction_pv_gnns(est_state_next, sqrtP_next, Z, sqrtR);
%         end
%     end
    
    
    %% postproc
    curr_ctrl = next_ctrl;
    curr_state = next_state;
    curr_state_dot = next_state_dot;
    est_state_curr = est_state_next;
    sqrtP_curr = sqrtP_next;
    
    est_states(:, i) = est_state_curr;
    act_states(:, i) = full_state_curr;
    mes_states(:, i) = mes_state_curr;
    sqrtP_diag(:,i) = diag(sqrtP_next);
    sqrtR_diag(:,i) = diag(sqrtR);

end

