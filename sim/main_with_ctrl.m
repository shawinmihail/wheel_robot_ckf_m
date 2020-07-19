clc
clear
close all

% TODO
% correction only vector part of quaternion
% note imu sensor has bias and attachment shift
% correct q with gps need to calculate another parts of accelarations


%% initial values and constants
%rand
seed = 200;
rng(seed);

% loop
dt = 10e-4;
N = 25000;

% surf
[surf_fcn, grad_surf] = custom_surf();

% initial
initial_x = 0.5;
initial_y = 3;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = 0/3;
initial_q = calc_q_full(grad_surf, initial_r, initial_yaw);
initial_w = [0;0;0];
initial_state = [initial_r; initial_v; initial_q];
initial_ctrl = [0; 0];

%% scripts
run('sensors_init')
run('init_ekf')

%% preproc
curr_state = initial_state;
curr_yaw = initial_yaw;
curr_w = initial_w;
curr_ctrl = initial_ctrl;
est_state_curr = initial_est_state;
sqrtP_curr = initial_sqrtP;

%% splines
load('D:\!MATLAB\wheel_robot_ckf_m\ctrl\traj3d.mat')
traj = traj3d;
traj_spline_obj = cscvn(traj);
cfs = traj_spline_obj.coefs;
bks = traj_spline_obj.breaks;

% temp
u_last = 0;

%% sim
t = 0;
for i = 1:N
    
    i    
    
    %% ctrl
%     next_ctrl = [1 ; 0.01];
%     y = curr_state(1:3);

    y = est_state_curr(1:3);
    y_ = curr_state(1:3);
    q = curr_state(7:10);
    C = quat2matrix(q);
    v = 1;
    [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, cfs, bks);
    [u_, sstar_, pstar_, DELTA_] = calculate_ctrl_3d(y_, v, C, cfs, bks);
    


    u_dot = 0;
    if norm(DELTA_) < 999
        u_dot = (u - u_last) / dt;
        lim = 10;
        u_dot = min(lim, max(-lim, u_dot));
        u = u_last + u_dot * dt;
        u_last = u;
    end
    if norm(DELTA_) < 0.1
        u_dot = (u - u_last) / dt;
        lim = 1;
        u_dot = min(lim, max(-lim, u_dot));
        u = u_last + u_dot * dt;
        u_last = u;
    end

    
    us(:, i) = u;
    sstars(: ,i) = u_dot;
    pstars(:, i) = pstar;
    deltas(:, i) = C' * DELTA_;
    deltas_norm(:, i) = norm(DELTA_(1:2));
    next_ctrl = [v ; u];
    
    %% wheel robot state evolution
    % state = [r v q]
    [next_state, next_yaw] = calculate_next_state(curr_state, curr_yaw, next_ctrl, grad_surf, dt);    
    [next_a, next_w] = calculate_acc_rotrate(curr_state, next_state, dt);
    next_w_dot = (next_w - curr_w) / dt;
    
    %% full state = [r v a q w]
    full_state_curr = [next_state(1:6);next_a;next_state(7:10);next_w; next_w_dot];

    %% mes_state
    mes_state_curr = mes_state_from_full_state(full_state_curr, ...
    gps_pos_local_rsm, gps_vel_local_rsm, gps_quat_rsm, gps_attachment_r, ...
    gps_slave_1, gps_slave_2, ...
    imu_acc_rsm, imu_acc_scale_factor, imu_acc_bias, ... 
    imu_rotvel_rsm, imu_rotvel_scale_factor, imu_rotvel_bias, imu_quat_shift);
     
    %% estimation, X = [r v q]
    a_mes = mes_state_curr(7:9);
    w_mes = mes_state_curr(14:16);
    
    %% predict with imu
    [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_curr, sqrtP_curr, Q, a_mes, w_mes, dt);
    
    %% correct pos vel gnns
%     if (mod(i, 50) == 5)
    Z = mes_state_curr(1:6);
    [est_state_next, sqrtP_next] = ekf_wr_correction_pv_gnns(est_state_next, sqrtP_next, Z, sqrtR_pv_gnns, gps_attachment_r);
%     end
     
    %% correct vel abs and dir gnns
    [est_state_next, sqrtP_next] = ...
        ekf_wr_correction_v_abs_and_dir_gnns(est_state_next, sqrtP_next, mes_state_curr(4:6), sqrtR_v_ad_gnns, gps_attachment_r);
     
    %% correct a
%     Z = mes_state_curr(7:9);
%     [est_state_next, sqrtP_next] = ekf_wr_correction_a_imu(est_state_next, sqrtP_next, Z, sqrtR_g_imu);
    
    %% correct pos vel att gnns
    dp1 = mes_state_curr(17:19);
    dp2 = mes_state_curr(20:22);
    dr1 = gps_slave_1;
    dr2 = gps_slave_2;
    Z = [dp1; dp2];
    [est_state_next, sqrtP_next] = ekf_wr_correction_p3_gnns(est_state_next, sqrtP_next, Z, sqrtR_p3_gnns, dr1, dr2);

    
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

figure
hold on
title('us')
plot(timeline, us)

figure
hold on
title('sstars')
plot(timeline, sstars)

figure
hold on
plot(timeline, deltas(1, :), 'r')
plot(timeline, deltas(2, :), 'g')
plot(timeline, deltas(3, :), 'b')

figure
hold on
fs = 20;
set(gca,'FontSize',fs)
title('\delta(t)')
xlabel('t, s')
ylabel('\delta, m')
plot(timeline, deltas_norm, 'r')

figure
hold on
grid on
plot3(traj3d(1, :), traj3d(2, :), traj3d(3, :),'b*')

splines_length = length(cfs(:, 1)) / 3;
for i = 1:splines_length

n = (i-1)*3 + 1;
spline_cfs = [cfs(n, 1) cfs(n, 2) cfs(n, 3) cfs(n, 4);
                 cfs(n+1, 1) cfs(n+1, 2) cfs(n+1, 3) cfs(n+1, 4);
                 cfs(n+2, 1) cfs(n+2, 2) cfs(n+2, 3) cfs(n+2, 4)];

for S = 0:0.1:bks(i+1)-bks(i)
    point = spline_point_3d(spline_cfs, S);
    plot3(point(1), point(2), point(3), 'r.')
end

end

plot3(act_states(1,:),act_states(2,:),act_states(3,:), 'k--');


