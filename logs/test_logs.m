clc
clear
close all

%% initial values and constants

%% scripts
run('prepare_log_data')
run('sensors_init')
run('init_ekf')

%% preproc
est_state_next = initial_est_state;
sqrtP_next = initial_sqrtP;

%% sim
T = t_imu(end);
step = 1e-2;
i = 0;
for t = 0:step:T
    
    imu_event = find((t+step)>t_imu & t_imu>t);
    gps_event = find((t+step)>t_gps & t_gps>t);
    
    %% predict with imu
    if ~isempty(imu_event)
        i = i + 1;
        if i <= 1
            continue
        end
        dt = imu_data(imu_event) - imu_data(imu_event - 1);
        a_mes = ned2enu(imu_data(imu_event-1, 2:4)');
        w_mes = ned2enu(imu_data(imu_event-1, 5:7)');
        [est_state_next, sqrtP_next] = ekf_wr_prediction_imu(est_state_next, sqrtP_next, Q, a_mes, w_mes, dt);
        
        est_state_next = est_state_next;
        sqrtP_next = sqrtP_next;
        est_states(:, i) = est_state_next;
        sqrtP_diag(:,i) = diag(sqrtP_next);
        timeline(i) = t;
    end
    
    if ~isempty(gps_event)
%         t
    end
    

end

figure
hold on
plot(ax)
plot(ay)
plot(az)
