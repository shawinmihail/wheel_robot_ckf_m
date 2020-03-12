%% gps
gps_pos_local_rsm = 0.12;
gps_vel_local_rsm = 0.05;
gps_quat_rsm = 0.001;
gps_attachment_r = [0.3; -0.3; 0.9];

%% imu
imu_attachment_r = [0; 0; 0]; %% imu is target point for simplification
imu_quat_shift = [100; 0; 0; 0];
imu_quat_shift = imu_quat_shift / norm(imu_quat_shift);
% acc
imu_acc_rsm = 0.05;
imu_acc_scale_factor = [1.00 0      0;
                        0    1.00   0;
                        0    0     1.00];
imu_acc_bias = [0.00; 0.00; 0.00];

% gyro
imu_rotvel_rsm = 0.001;
imu_rotvel_scale_factor = [1.00 0      0;
                           0    1.00   0;
                           0    0     1.00];
imu_rotvel_bias = [0.01; 0.00; 0.00];