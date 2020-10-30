function [X, sqrtP] = ekf3_wr_correct_a_gnns(X, sqrtP, Z, R, dr_imu_gnns)

%% X [r v a q w]
g = [0;0;-9.72];
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% dv_gnss/dt = a_gnns = a - dr_gnns * w_dot - w cross (dr_gnss cross w);
% w_dot is difficult to find, then
Z_x = a + quatRotate(q, cross(w, cross(dr_imu_gnns, w)));
dz = Z - Z_x;


%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

v = cross(w, cross(dr_imu_gnns, w));
Z_agnns_q = quat_rot_fcn_j(q(1),q(2),q(3),q(4),v(1),v(2),v(3));
Z_agnns_w = quat2matrix(q) * (x_operator(w)*x_operator(dr_imu_gnns) - x_operator(cross(dr_imu_gnns, w)));
H = [O33 O33 E33 Z_agnns_q  Z_agnns_w];

%% ordinary K, H
P = sqrtP * sqrtP';
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
sqrtP = chol(P,'lower');
X = X + K*dz;
X(10:13) = X(10:13) / norm(X(10:13));

end

