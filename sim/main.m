clc
clear
close all

% TODO
% correction only vector part of quaternion
% quat mean and covariance -- find right way to define
% note imu sensor has bias and attachment shift
% correct q with gps need to calculate another parts of accelarations
% ekf_wr_correction_v_unit_gnns v/norm(v) -- can be added to H


%% initial values and constants
%rand
seed = 200;
rng(seed);

% loop
dt = 1e-2;
N = 19999;

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
run('estimation_init')

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
    if t < 3
        next_ctrl = [0 ; 0.01 - 0.001*t];
    else
        next_ctrl = [1 ; 0];   
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
    [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_curr, sqrtP_curr, Q, a_mes, w_mes, dt);

    %% correct pos vel gnns
    if i > -1
        if (mod(i, 50) == 1)
            if rand() > -1
                Z = mes_state_curr(1:6);
                [est_state_next, sqrtP_next] = ekf_wr_correction_pv_gnns(est_state_next, sqrtP_next, Z, sqrtR_pv_gnns, gps_attachment_r);
            end
        end
    end
    
    %% correct vel abs and dir gnns
    if i > -1
        if (mod(i, 10) == 1)
            if rand() > -1
                v = mes_state_curr(4:6);
                nv = norm(v);
                if nv > -1
                    Z = v;
                    [est_state_next, sqrtP_next] = ...
                        ekf_wr_correction_v_abs_and_dir_gnns(est_state_next, sqrtP_next, Z, sqrtR_v_ad_gnns, gps_attachment_r);
                end
            end
        end
    end
        
    %% correct att g imu
    if i < -1
        if (mod(i, 10) == 1)
            if rand() > -1
                Z = mes_state_curr(7:9);
                [est_state_next, sqrtP_next] = ekf_wr_correction_a_imu(est_state_next, sqrtP_next, Z, sqrtR_g_imu);
            end
        end
    end
    
%     %% correct pos vel att gnns
%     if (mod(i, 150) == 99)
%         if rand() > 0.1
%             Z = [mes_state_curr(1:6); mes_state_curr(10:13)];
%             [est_state_next, sqrtP_next] = ekf_wr_correction_pvq_gnns(est_state_next, sqrtP_next, Z, sqrtR_pvq_gnns);
%         end
%     end
    
    
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




