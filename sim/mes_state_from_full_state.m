function mes_state = mes_state_from_full_state(...
    full_state, gps_pos_local_rsm, gps_vel_local_rsm, gps_quat_rsm, imu_acc_rsm, imu_rot_vel_rsm)

%% env
g = [0 0 -10]';

%% add noise
mes_state = zeros(16, 1);
% r
mes_state(1:3) = full_state(1:3) + randn(3, 1) * gps_pos_local_rsm;
% v
mes_state(4:6) = full_state(4:6) + randn(3, 1) * gps_vel_local_rsm;
% a
q = full_state(10:13);
a = full_state(7:9) - g;
aB = quatRotate(quatDual(q), a);
mes_state(7:9) = aB + randn(3, 1) * imu_acc_rsm;
% q
mes_state(10:13) = full_state(10:13) + randn(4, 1) * gps_quat_rsm;
mes_state(10:13) = mes_state(10:13) / norm(mes_state(10:13));
% w
mes_state(14:16) = full_state(14:16) + 0 * randn(3, 1) * imu_rot_vel_rsm;
