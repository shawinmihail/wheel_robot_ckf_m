% initial_est_state = [initial_state(1:6);[0;0;0];initial_state(7:10);[0;0;0]];
initial_est_state = [[0;0;0];[0;0;0];[0;0;0];[1;0;0;0];[0;0;0]];

R_pv_gnns = diag([gps_pos_local_rsm*[1; 1; 1]; gps_vel_local_rsm*[1; 1; 1]]).^2;
R_g_imu = diag(imu_acc_rsm*[1; 1; 1]).^2;
R_v_ad_gnns = diag(gps_vel_local_rsm*[1; 1; 1]).^2;
R_p3_gnns = diag(gps_pos_local_rsm*[1; 1; 1; 1; 1; 1]).^2; 
Q = diag([1e-2*[1; 1; 1]; 1e-2*[1; 1; 1]; 1e-2*[1; 1; 1]; 1e-4*[1; 1; 1; 1]; 1e-2*[1; 1; 1]]);
P0 = 10*Q;

sqrtR_pv_gnns = chol(R_pv_gnns,'lower');
sqrtR_g_imu = chol(R_g_imu,'lower');
sqrtR_v_ad_gnns= chol(R_v_ad_gnns,'lower');
sqrtR_p3_gnns= chol(R_p3_gnns,'lower');
initial_sqrtP = chol(P0,'lower');


