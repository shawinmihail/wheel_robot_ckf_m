clc
clear

q = sym('q', [4, 1]);
a = sym('a', [3, 1]);
g = sym('g', [3, 1]);
w = sym('w', [3, 1]);
dr = sym('dr', [3, 1]);
assume([q; a; g; w; dr], 'real')

%% 
imu_a_q(q) = quatRotate(quatDual(q), a - g) - cross(w, cross(dr, w));
imu_a_w(w) = quatRotate(quatDual(q), a - g) - cross(w, cross(dr, w));
imu_a_a(a) = quatRotate(quatDual(q), a - g) - cross(w, cross(dr, w));

imu_a_q_j = jacobian(imu_a_q, q)
imu_a_w_j = jacobian(imu_a_w, w)
imu_a_a_j = jacobian(imu_a_a, a)


matlabFunction(imu_a_q_j,'file','ekf/ekf_routins/Z_aimu_dq_fcn.m')
matlabFunction(imu_a_w_j,'file','ekf/ekf_routins/Z_aimu_dw_fcn.m')
matlabFunction(imu_a_a_j,'file','ekf/ekf_routins/Z_aimu_da_fcn.m')


