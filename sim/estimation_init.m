initial_est_state = [initial_state(1:6);[0;0;0];initial_state(7:9);[0;0;0]];

R_pv_gnns = diag([gps_pos_local_rsm*[1; 1; 1]; gps_vel_local_rsm*[1; 1; 1]]).^2;
R_g_imu = diag(imu_acc_rsm*[1; 1; 1]).^2;
R_pvq_gnns = diag([gps_pos_local_rsm*[1; 1; 1]; gps_vel_local_rsm*[1; 1; 1]; gps_quat_rsm*[1; 1; 1; 1]]).^2;
R_v_unit_gnns = diag(gps_vel_local_rsm*[1; 1; 1]).^2;

Q = diag([1e-4*[1; 1; 1]; 1e-4*[1; 1; 1]; 1e-4*[1; 1; 1]; 1e-8*[1; 1; 1; 1]; 1e-4*[1; 1; 1]]);
P0 = 25*Q;

R_att = diag(1e-6*[1 1 1 1]);

sqrtR_pv_gnns = chol(R_pv_gnns,'lower');
sqrtR_pvq_gnns = chol(R_pvq_gnns,'lower');
sqrtR_g_imu = chol(R_g_imu,'lower');
sqrtR_v_unit_gnns= chol(R_v_unit_gnns,'lower');
sqrtQ = chol(Q,'lower');
initial_sqrtP = chol(P0,'lower');


