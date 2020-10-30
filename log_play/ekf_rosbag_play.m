% clc
% clear
% close all

%% initial values and constants
load('log_play/data_prepared/ts_imu.mat')
load('log_play/data_prepared/array_imu.mat')

load('log_play/data_prepared/array_triplet.mat')
load('log_play/data_prepared/ts_triplet.mat')

load('log_play/data_prepared/array_steering.mat')
load('log_play/data_prepared/ts_steering.mat')


%% init sensors
% imu
load('log_play/data_prepared/g_calib.mat')
load('log_play/data_prepared/w_calib.mat')
% gps
load('log_play/data_prepared/slave1_dr.mat')
load('log_play/data_prepared/slave2_dr.mat')
load('log_play/data_prepared/wgs_ref.mat')
lat_ref = wgs_ref(1);
lon_ref = wgs_ref(2);
alt_ref = wgs_ref(3);
[x_ref, y_ref, z_ref] = wgs2ecef(lat_ref, lon_ref, alt_ref);
r_ref = [x_ref; y_ref; z_ref];
R_ref = rmx2enu(lat_ref, lon_ref);
gps_slave_1_enu = R_ref * slave1_dr';
gps_slave_2_enu = R_ref * slave2_dr';

% slaves center in ex
d = gps_slave_1_enu/2 + gps_slave_2_enu/2;
dh = d.*[1 1 0]';
q = quatBetweenVectors(dh, [1;0;0]);
gps_slave_1_enu = quatRotate(q, gps_slave_1_enu);
gps_slave_2_enu = quatRotate(q, gps_slave_2_enu);

%% setup_ekf !!!!!!!
run('init_ekf3')

%% preproc
est_state_next = initial_est_state;
sqrtP_next = initial_sqrtP;

%% sim
i = 0;
est_states = [];
timeline = [];

estimated_array_triplet = [];
estimated_array_imu = [];

enu_timeline = [];
v_mes_enu_list = [];
r_mes_enu_list = [];
a_mes_enu_list = [];
dr1_mes_enu_list = [];
dr2_mes_enu_list = [];

T0 = 110;
% T0 = ts_triplet(1);
T = 160;
% T = ts_triplet(end);
step = 50e-3;
% assume target point is imu attachment point
dr_imu_gnns = [-0.3; 0.0; 0.3];
dr_bshc_gnns = [0.0; 0.0; 0.6]; % back shaft center -> gnns
for t = T0:step:T
    
    [isevent_imu, stamp_imu, measure_imu] = measured_event(t, t-step, ts_imu, array_imu);
    [is_event_triplet, stamp_triplet, measure_triplet] = measured_event(t, t-step, ts_triplet, array_triplet);
%     [is_event_steering, stamp_steering, measure_steering] = measured_event(t, t-step, ts_steering, array_steering);

    %% predict
    [est_state_next, sqrtP_next] = ekf3_wr_predict(est_state_next, sqrtP_next, Q, step);
        
    %% correct with imu
    if isevent_imu
        a_mes = measure_imu(1:3)'; %???
        a_mes = [-a_mes(2);a_mes(1);a_mes(3)];
        w_mes = measure_imu(4:6)'-w_calib; %???
        w_mes = [-w_mes(2);w_mes(1);w_mes(3)];
        
        [est_state_next, sqrtP_next] = ekf3_wr_corrrect_a_imu(est_state_next, sqrtP_next, a_mes, R_a_imu, g_calib);
        [est_state_next, sqrtP_next] = ekf3_wr_corrrect_w_imu(est_state_next, sqrtP_next, w_mes, R_a_imu, g_calib);
        
    end
    
    if is_event_triplet
        k0 = measure_triplet(13);
        k1 = measure_triplet(14);
        k2 = measure_triplet(15);
        
        r_mes = measure_triplet(1:3)';
        v_mes = measure_triplet(4:6)';
        v_mes_enu = R_ref * v_mes;
        r_mes_enu = R_ref * (r_mes - r_ref);
        dr_s1_mes = measure_triplet(7:9)';
        dr_s2_mes = measure_triplet(10:12)';
        dr1_mes_enu = R_ref * dr_s1_mes;
        dr2_mes_enu = R_ref * dr_s2_mes;

        % save
        enu_timeline = [enu_timeline t];
        r_mes_enu_list = [r_mes_enu_list r_mes_enu];
        v_mes_enu_list = [v_mes_enu_list v_mes_enu];
        dr1_mes_enu_list = [dr1_mes_enu_list dr1_mes_enu];
        dr2_mes_enu_list = [dr2_mes_enu_list dr2_mes_enu];
        a_gnns = zeros(3,1);
        if length(v_mes_enu_list(1,:)) > 1
            a_gnns = (v_mes_enu(:,end) - v_mes_enu_list(:,end-1)) / (enu_timeline(end) - enu_timeline(end-1));
        end
        a_mes_enu_list = [a_mes_enu_list a_gnns];
        %
        
        %% correct a_gnns
        Z = a_gnns;
        [est_state_next, sqrtP_next] = ekf3_wr_correct_a_gnns(est_state_next, sqrtP_next, Z, R_a_gnns, dr_imu_gnns);
        
        %% correct pos vel gnns
        if k0 == 4             
            Z = [r_mes_enu; v_mes_enu];
            [est_state_next, sqrtP_next] = ekf3_wr_correct_rv_gnns(est_state_next, sqrtP_next, Z, R_rv_gnns, dr_imu_gnns);
        end

        %% correct vel abs and dir gnns
        r_mes = measure_triplet(1:3)';
        v_mes = measure_triplet(4:6)';

        v_mes_enu = R_ref * v_mes;
        Z = v_mes_enu;
        [est_state_next, sqrtP_next] = ekf3_wr_correct_u(est_state_next, sqrtP_next, Z, R_u_gnns, dr_bshc_gnns);

        % correct pos vel att gnns
        if k1 == 4 && k2 == 4
            Z = [dr1_mes_enu; dr2_mes_enu];
%             [est_state_next, sqrtP_next] = ekf3_wr_correct_q2_gnns(est_state_next, sqrtP_next, Z, R_q2_gnns, gps_slave_1_enu, gps_slave_2_enu);
        end
       
    end
    
    est_states = [est_states est_state_next];
    
    % repair mes state
    timeline = [timeline t];
    q = est_state_next(10:13);
    r = est_state_next(1:3);
    v = est_state_next(4:6);
    a = est_state_next(7:9);
    q = est_state_next(10:13);
    w = est_state_next(14:16);
    
    estimated_a_mes = quatRotate(quatDual(q), est_state_next(7:9) - g_calib);
    estimated_a_mes = [estimated_a_mes(2);-estimated_a_mes(1);estimated_a_mes(3)];
    estimated_w_mes = w + w_calib;
    estimated_w_mes = [estimated_w_mes(2);-estimated_w_mes(1);estimated_w_mes(3)];
    estimated_a_gnns = a + quatRotate(q, cross(w, cross(dr_imu_gnns, w)));
    estimated_imu = [estimated_a_mes' estimated_w_mes' estimated_a_gnns'];
    estimated_array_imu = [estimated_array_imu; estimated_imu];
    
    estimated_r_base = r + quatRotate(q, dr_imu_gnns);
    
    estimated_v_base = v + quatRotate(q, cross(w, dr_imu_gnns)); %000
    estimated_u_base = quatRotate(q, [1;0;0] * norm(v)) + quatRotate(q, cross(w, dr_bshc_gnns)); % 000
    
    estimated_r_slave1 = quatRotate(q, gps_slave_1_enu);
    estimated_r_slave2 = quatRotate(q, gps_slave_2_enu);
    estimated_triplet = [estimated_r_base' estimated_v_base' estimated_r_slave1' estimated_r_slave2' estimated_u_base'];
    estimated_array_triplet = [estimated_array_triplet; estimated_triplet];
end

% mes_q = est_states(10:13, :);
% save('logs/mes_q', 'mes_q')

% qtest
