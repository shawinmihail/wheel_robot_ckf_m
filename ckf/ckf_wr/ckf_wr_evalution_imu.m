function X = ckf_wr_evalution_imu(X0, a_mes, w_mes, dt)

q0 = X0(7:10);
g = [0;0;0];
a0 = quatRotate(q0,a_mes) + g;
w0 = w_mes;
% Euler
X_dot = ckf_wr_model_imu(X0, a0, w0);
X = X0 + X_dot * dt;

% RK4
% Y_dot_1 = imu_meas_body_dyn(Y0, a0, w0);
% Y_dot_2 = imu_meas_body_dyn(Y0 + 0.5 * Y_dot_1 * dt, a0, w0);
% Y_dot_3 = imu_meas_body_dyn(Y0 + 0.5 * Y_dot_2 * dt, a0, w0);
% Y_dot_4 = imu_meas_body_dyn(Y0 + 1.0 * Y_dot_3 * dt, a0, w0);
% Y = Y0 + (1/6)*(Y_dot_1 + 2*Y_dot_2 + 2*Y_dot_3 + Y_dot_4)*dt;

q = X(7:10);
q_n = norm(q);
q = q / q_n;
X(7:10) = q; 
