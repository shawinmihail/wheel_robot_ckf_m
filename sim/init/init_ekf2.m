% initial_est_state = [initial_state(1:6); [0;0;0]; initial_state(7:10)];
initial_est_state = [[0;0;0];[0;0;0];[0;0;0];[1;0;0;0]];

R_p_gnns = diag(gps_pos_local_rsm*[1; 1; 1]).^2;
R_v_gnns = diag(gps_vel_local_rsm*[1; 1; 1]).^2;
R_u_gnns = diag(gps_vel_local_rsm*[1; 1; 1]).^2;
R_a_imu = diag(imu_acc_rsm*[1; 1; 1]).^2;

Q = diag([1e-3*[1; 1; 1]; 1e-3*[1; 1; 1]; 1e-3*[1; 1; 1]; 1e-5*[1; 1; 1; 1]]);
P0 = 10*Q;

sqrtR_p_gnns = chol(R_p_gnns,'lower');
sqrtR_v_gnns = chol(R_v_gnns,'lower');
sqrtR_u_gnns = chol(R_u_gnns,'lower');
sqrtR_a_imu = chol(R_a_imu,'lower');
initial_sqrtP = chol(P0,'lower');


