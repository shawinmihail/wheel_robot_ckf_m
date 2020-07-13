% initial_est_state = [initial_state(1:6);[0;0;0];initial_state(7:10);[0;0;0]];
q = [100;0;0;0];
q = q / norm(q);
initial_est_state = [[0;0;0];[0;0;0];[0;0;0];q;[0;0;0]];

R_pv_gnns = diag([1e-4*[1; 1; 1]; 1e-4*[1; 1; 1]]);
R_g_imu = diag(1e-1*[1; 1; 1]);
R_v_ad_gnns = diag(1e-4*[1; 1; 1]);
R_p3_gnns = diag(1e-7*[1; 1; 1; 1; 1; 1; 1; 1; 1]);
R_p2_gnns = diag(1e-7*[1; 1; 1; 1; 1; 1]); 


Q = diag([1e-1*[1; 1; 1]; 1e-1*[1; 1; 1]; 1e-1*[1; 1; 1]; 1e-3*[1; 1; 1; 10]; 1e-4*[1; 1; 1]]);
P0 = 30*Q;

sqrtR_pv_gnns = chol(R_pv_gnns,'lower');
sqrtR_g_imu = chol(R_g_imu,'lower');
sqrtR_v_ad_gnns= chol(R_v_ad_gnns,'lower');
sqrtR_p3_gnns= chol(R_p3_gnns,'lower');
sqrtR_p2_gnns= chol(R_p2_gnns,'lower');
initial_sqrtP = chol(P0,'lower');