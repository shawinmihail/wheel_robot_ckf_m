function mes_state = mes_state_from_full_state(full_state, ...
    gps_pos_local_rsm, gps_vel_local_rsm, gps_quat_rsm, gps_attachment_r, ...
    imu_acc_rsm, imu_acc_scale_factor, imu_acc_bias, ... 
    imu_rotvel_rsm, imu_rotvel_scale_factor, imu_rotvel_bias, imu_quat_shift)

q = full_state(10:13);

%% env
g = [0 0 -10]';

%% add noise
mes_state = zeros(16, 1);
% r
dr = quatRotate(q ,gps_attachment_r);
mes_state(1:3) = full_state(1:3) + dr + randn(3, 1) * gps_pos_local_rsm;

% v
dv = quatRotate(q, cross(full_state(14:16), gps_attachment_r));
mes_state(4:6) = full_state(4:6) + dv + randn(3, 1) * gps_vel_local_rsm;

% a
a_tr = quatRotate(quatDual(q), imu_acc_scale_factor*(full_state(7:9) - g));
mes_state(7:9) = quatRotate(imu_quat_shift, a_tr + randn(3, 1) * imu_acc_rsm + imu_acc_bias);
% q
mes_state(10:13) = full_state(10:13) + randn(4, 1) * gps_quat_rsm;
mes_state(10:13) = mes_state(10:13) / norm(mes_state(10:13));
% w
mes_state(14:16) = imu_rotvel_scale_factor * full_state(14:16)  + randn(3, 1) * imu_rotvel_rsm + imu_rotvel_bias;
