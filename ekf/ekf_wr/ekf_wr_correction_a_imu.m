function [X, sqrtP] = ekf_wr_correction_a_imu(X, sqrtP, Z, sqrtR, imu_attachment_r)
n = length(X);
m = length(Z);

%% X [r v a q w]
g = [0;0;-10];
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [a_imu]
% a_imu = quatRotate(quatDual(q), a - g) - cross(w, cross(dr, w));
Z_x = quatRotate(quatDual(q), a - g) - cross(w, cross(imu_attachment_r, w));
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zaa = Z_aimu_da_fcn(a(1),a(2),a(3),q(1),q(2),q(3),q(4));
Zaq = Z_aimu_dq_fcn(q(1),q(2),q(3),q(4),a(1),a(2),a(3),g(1),g(2),g(3));
Zaw = Z_aimu_dw_fcn(w(1),w(2),w(3),imu_attachment_r(1),imu_attachment_r(2),imu_attachment_r(3));
H = [O33 O33 Zaa Zaq Zaw];

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;


end
