% x = [s e q]
imu_calib_state = [[1;1;1]; [0;0;0]; [1;0;0;0]];
R_imu_calib = diag(1e-2 * [1; 1; 1]);

Q_imu_calib = diag([1e-8*[1; 1; 1]; 1e-8*[1; 1; 1]; 1e-3*[1; 1; 1; 1]]);
P_imu_calib = 10*Q_imu_calib;

sqrtR_imu_calib  = chol(R_imu_calib,'lower');
sqrt_P_imu_calib = chol(P_imu_calib,'lower');


