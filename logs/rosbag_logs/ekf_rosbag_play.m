clc
clear
close all

%% initial values and constants
load('D:\!MATLAB\wheel_robot_ckf_m\logs\ts_imu.mat')
load('D:\!MATLAB\wheel_robot_ckf_m\logs\array_imu.mat')

load('D:\!MATLAB\wheel_robot_ckf_m\logs\array_triplet.mat')
load('D:\!MATLAB\wheel_robot_ckf_m\logs\ts_triplet.mat')

load('D:\!MATLAB\wheel_robot_ckf_m\logs\array_steering.mat')
load('D:\!MATLAB\wheel_robot_ckf_m\logs\ts_steering.mat')


%% init sensors
% gps
gps_attachment_r = [-0.1; -0.1; 0.35]; % ????
load('D:\!MATLAB\wheel_robot_ckf_m\logs\slave1_dr.mat')
load('D:\!MATLAB\wheel_robot_ckf_m\logs\slave2_dr.mat')
load('D:\!MATLAB\wheel_robot_ckf_m\logs\wgs_ref.mat')
lat_ref = wgs_ref(1);
lon_ref = wgs_ref(2);
alt_ref = wgs_ref(3);
[x_ref, y_ref, z_ref] = wgs2ecef(lat_ref, lon_ref, alt_ref);
r_ref = [x_ref; y_ref; z_ref];
R_ref = rmx2enu(lat_ref, lon_ref);
gps_slave_1 = slave1_dr';
gps_slave_2 = slave2_dr';


%% setup_ekf
run('init_ekf_logs')

%% preproc
est_state_next = initial_est_state;
sqrtP_next = initial_sqrtP;

%% sim
i = 0;
est_states = [];
timeline = [];
est_gps_slaves1 = [];
est_gps_slaves2 = [];
est_imu_a = [];

enu_timeline = [];
v_mes_enu_list = [];
r_mes_enu_list = [];

T0 = 0;
T = 20;
step = 1e-2;

a_mes = [array_imu(1,1);array_imu(1,2);array_imu(1,3)];
w_mes = [0;0;0];
for t = T0:step:T
    
    [isevent_imu, stamp_imu, measure_imu] = measured_event(t, t-step, ts_imu, array_imu);
    [is_event_triplet, stamp_triplet, measure_triplet] = measured_event(t, t-step, ts_triplet, array_triplet);
%     [is_event_steering, stamp_steering, measure_steering] = measured_event(t, t-step, ts_steering, array_steering);
    
    %% predict with imu
    [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_next, sqrtP_next, Q, a_mes, w_mes, step);
    
    %% correct with imu
    if isevent_imu
        
        a_mes = measure_imu(1:3)'; %???
        w_mes = measure_imu(4:6)'; %???
         
        %% correct a
        Z = a_mes;
        [est_state_next, sqrtP_next] = ekf_wr_correction_a_imu(est_state_next, sqrtP_next, Z, sqrtR_g_imu);
    end
    
    if is_event_triplet
        k0 = measure_triplet(13);
        k1 = measure_triplet(14);
        k2 = measure_triplet(15);
        
        r_mes = measure_triplet(1:3)';
        v_mes = measure_triplet(4:6)';
        v_mes_enu = R_ref * v_mes;
        r_mes_enu = R_ref * (r_mes - r_ref);

        % save
        enu_timeline = [enu_timeline t];
        r_mes_enu_list = [r_mes_enu_list r_mes_enu];          
        v_mes_enu_list = [v_mes_enu_list v_mes_enu];
        %
        
        %% correct pos vel gnns
        if k0 == 4             
            Z = [r_mes_enu; v_mes_enu];
            [est_state_next, sqrtP_next] = ekf_wr_correction_pv_gnns(est_state_next, sqrtP_next, Z, sqrtR_pv_gnns, gps_attachment_r);
        end

        %% correct vel abs and dir gnns
        r_mes = measure_triplet(1:3)';
        v_mes = measure_triplet(4:6)';

        v_mes_enu = R_ref * v_mes;
        Z = v_mes_enu;
        [est_state_next, sqrtP_next] = ...
            ekf_wr_correction_v_abs_and_dir_gnns(est_state_next, sqrtP_next, Z, sqrtR_v_ad_gnns, gps_attachment_r);

        %% correct pos vel att gnns
%         if k1 == 4 && k2 == 4
%             dp1 = gps_mes(measure_triplet, 7:9)';
%             dp2 = gps_mes(measure_triplet, 10:12)';
%             dp3 = dp2 - dp1;
%             dr1 = gps_slave_1;
%             dr2 = gps_slave_2;
%             dr3 = gps_slave_2 - gps_slave_1;
%             Z = [dp1; dp2];
%             [est_state_next, sqrtP_next] = ekf_wr_correction_p2_gnns(est_state_next, sqrtP_next, Z, sqrtR_p2_gnns, dr1, dr2);
% 
%             q = est_state_next(10:13);
%             est_gps_slave1 = quatRotate(q, gps_slave_1);
%             est_gps_slave2 = quatRotate(q, gps_slave_2);
%         end
%         
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
