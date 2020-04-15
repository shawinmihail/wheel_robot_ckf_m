clc
clear
close all

%% initial values and constants

%% scripts
% run('plot_logs')
% clc
% clear
% close all

load('wheel_robot_ckf_m\logs\gps_mes0.mat')
load('wheel_robot_ckf_m\logs\imu_mes0.mat')

T0 = 0;
T = 70;
% T = gps_mes(end, 1);
time_cutter_imu = find(imu_mes(:, 1) < T & imu_mes(:, 1) > T0);
imu_mes = imu_mes(time_cutter_imu(1):time_cutter_imu(end), :);
time_cutter_gps = find(gps_mes(:, 1) < T & gps_mes(:, 1) > T0);
gps_mes = gps_mes(time_cutter_gps(1):time_cutter_gps(end), :);

%% init sensors
% gps
gps_attachment_r = [0.0; 0.0; 0.0]; % ????
gps_slave_1 = [-0.76; -0.23; 0.19];
gps_slave_2 = [-0.76;  0.23; 0.19];


%% setup_ekf
run('init_ekf_logs')

%% preproc
est_state_next = initial_est_state;
sqrtP_next = initial_sqrtP;

%% sim
t_imu = imu_mes(:,1);
t_gps = gps_mes(:,1);
t0 = t_imu(1);
step = 1e-2;
i = 0;
est_states = [];
timeline = [];
est_gps_slaves1 = [];
est_gps_slaves2 = [];
est_imu_a = [];

array1 = [];
for t = T0:step:floor(T)
    
    imu_events = find((t+step)>t_imu & t_imu>t);
    gps_events = find((t+step) > t_gps & t < t_gps);
    
    if ~isempty(imu_events)
        dt = t - t0;
        
        %% predict with imu
        a_mes = frb2flt(imu_mes(imu_events(1), 2:4)');
        w_mes = frb2flt(imu_mes(imu_events(1), 5:7)');
        [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_next, sqrtP_next, Q, a_mes, w_mes, dt);
        
        %% correct a
        a_mes = frb2flt(imu_mes(imu_events(1), 2:4)');
        Z = a_mes;
        [est_state_next, sqrtP_next] = ekf_wr_correction_a_imu(est_state_next, sqrtP_next, Z, sqrtR_g_imu);
        
        t0 = t;
    end
    
    if ~isempty(gps_events)
        dt = t - t0;
        k0 = gps_mes(gps_events(1), 14);
        k1 = gps_mes(gps_events(1), 15);
        k2 = gps_mes(gps_events(1), 16);
        
        %% correct pos vel gnns
        if k0 == 4
            Z = gps_mes(gps_events(1), 2:7)';
            [est_state_next, sqrtP_next] = ekf_wr_correction_pv_gnns(est_state_next, sqrtP_next, Z, sqrtR_pv_gnns, gps_attachment_r);
        end

        %% correct vel abs and dir gnns
        Z = gps_mes(gps_events(1), 5:7)';
        [est_state_next, sqrtP_next] = ...
            ekf_wr_correction_v_abs_and_dir_gnns(est_state_next, sqrtP_next, Z, sqrtR_v_ad_gnns, gps_attachment_r);

        %% correct pos vel att gnns
%         if k1 == 4 & k2 == 4
%             dp1 = gps_mes(gps_events(1), 8:10)';
%             dp2 = gps_mes(gps_events(1), 11:13)';
%             dp3 = dp2 - dp1;
%             dr1 = gps_slave_1;
%             dr2 = gps_slave_2;
%             dr3 = gps_slave_2 - gps_slave_1;
%             Z = [dp1; dp2; dp3];
%             [est_state_next, sqrtP_next] = ekf_wr_correction_p3_gnns(est_state_next, sqrtP_next, Z, sqrtR_p3_gnns, dr1, dr2, dr3);
% 
%             q = est_state_next(10:13);
%             est_gps_slave1 = quatRotate(q, gps_slave_1);
%             est_gps_slave2 = quatRotate(q, gps_slave_2);
% 
%             d1 = est_gps_slave1 - dp1;
%             array1 = [array1 d1];
%         end
        
        t0 = t;
    end
    
    est_states = [est_states est_state_next];
    timeline = [timeline t];
    q = est_state_next(10:13);
    
    est_gps_slave1 = quatRotate(q, gps_slave_1);
    est_gps_slave2 = quatRotate(q, gps_slave_2);
    
    est_gps_slaves1 = [est_gps_slaves1 est_gps_slave1];
    est_gps_slaves2 = [est_gps_slaves2 est_gps_slave2];
    
    g = [0;0;-9.8];
    est_imu_a_next = quatRotate(quatDual(q), est_state_next(7:9) - g);
    est_imu_a = [est_imu_a est_imu_a_next];
end

% mes_q = est_states(10:13, :);
% save('logs/mes_q', 'mes_q')
