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
N = 9999;

% surf
[surf_fcn, grad_surf] = custom_surf();

%initial
initial_x = 0;
initial_y = 0;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = 0;
initial_q = calc_q_full(grad_surf, initial_r, initial_yaw);

initial_state = [initial_r; initial_v; initial_q];
initial_ctrl = [0; 0];
initial_est_state = initial_state;


%% sensors
% gps
gps_pos_local_rsm = 0.12;
gps_vel_local_rsm = 0.05;
gps_quat_rsm = 0.001;
% imu
imu_acc_rsm = 0.05;
imu_rot_vel_rsm = 0.01;

R_pv_gnns = diag([gps_pos_local_rsm*[1; 1; 1]; gps_vel_local_rsm*[1; 1; 1]]).^2;
R_g_imu = diag(imu_acc_rsm*[1; 1; 1]).^2;
R_pvq_gnns = diag([gps_pos_local_rsm*[1; 1; 1]; gps_vel_local_rsm*[1; 1; 1]; gps_quat_rsm*[1; 1; 1; 1]]).^2;
R_v_unit_gnns = diag(gps_vel_local_rsm*[1; 1; 1]).^2;

Q = diag([1e-4*[1; 1; 1];    1e-4*[1; 1; 1];    1e-4*[1; 1; 1; 1]]);
P0 = 12*Q;

R_att = diag(1e-6*[1 1 1 1]);

sqrtR_pv_gnns = chol(R_pv_gnns,'lower');
sqrtR_pvq_gnns = chol(R_pvq_gnns,'lower');
sqrtR_g_imu = chol(R_g_imu,'lower');
sqrtR_v_unit_gnns= chol(R_v_unit_gnns,'lower');
sqrtQ = chol(Q,'lower');
initial_sqrtP = chol(P0,'lower');

%% preproc
curr_state = initial_state;
curr_yaw = initial_yaw;
curr_ctrl = initial_ctrl;
est_state_curr = initial_est_state;
sqrtP_curr = initial_sqrtP;


t = 0;
for i = 1:N
    
    i    
    
    %% actuators dyn modeling
    next_ctrl = [1,0.1];
    next_ctrl = process_control_input(curr_ctrl, next_ctrl, dt);
    
    %% wheel robot state evolution
    % state = [r v q]
    [next_state, next_yaw] = calculate_next_state(curr_state, curr_yaw, next_ctrl, grad_surf, dt);    
    [a, w] = calculate_acc_rotrate(curr_state, next_state, dt);
    
    
    %% full state = [r v a q w]
    full_state_curr = [next_state(1:6);a;next_state(7:10);w]; 

    %% mes_state
    mes_state_curr = mes_state_from_full_state(...
        full_state_curr, gps_pos_local_rsm, gps_vel_local_rsm, gps_quat_rsm, imu_acc_rsm, imu_rot_vel_rsm);
     
    %% estimation, X = [r v q]
    a_mes = mes_state_curr(7:9);
    w_mes = mes_state_curr(14:16);
    
    %% predict with imu
    [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_curr, sqrtP_curr, sqrtQ, a_mes, w_mes, dt);
%     
% 
%     %% correct pos vel gnns
%     if (mod(i, 50) == 40)
%         if rand() > 0.1
%             Z = mes_state_curr(1:6);
%             [est_state_next, sqrtP_next] = ekf_wr_correction_pv_gnns(est_state_next, sqrtP_next, Z, sqrtR_pv_gnns);
%         end
%     end
%     
%     %% correct att by vel dir
%     if i > 1000
%         if rand() > 0.33
%             v = mes_state_curr(4:6);
%             nv = norm(mes_state_curr(4:6));
%             if nv > 0.5
%                 Z = v / nv;
%                 [est_state_next, sqrtP_next] = ekf_wr_correction_v_unit_gnns(est_state_next, sqrtP_next, Z, sqrtR_v_unit_gnns);
%             end
%         end
%     end
%     
%     %% correct pos vel att gnns
%     if (mod(i, 150) == 99)
%         if rand() > 0.1
%             Z = [mes_state_curr(1:6); mes_state_curr(10:13)];
% %             [est_state_next, sqrtP_next] = ekf_wr_correction_pvq_gnns(est_state_next, sqrtP_next, Z, sqrtR_pvq_gnns);
%         end
%     end
%     
%     %% correct att g imu
%     if i > 1000
%         if rand() > 0.33
%             Z = mes_state_curr(7:9);
%             [est_state_next, sqrtP_next] = ekf_wr_correction_g_imu(est_state_next, sqrtP_next, Z, sqrtR_g_imu);
%         end
%     end
%     
%     
%     %% postproc
    curr_ctrl = next_ctrl;
    curr_state = next_state;
    curr_yaw = next_yaw;
    est_state_curr = est_state_next;
    sqrtP_curr = sqrtP_next;
%     
    est_states(:, i) = est_state_curr;
    act_states(:, i) = full_state_curr;
    mes_states(:, i) = mes_state_curr;
    sqrtP_diag(:,i) = diag(sqrtP_next);

t = t + dt;

end

%% plot




% figure
% hold on
% plot(aw(6, :))




