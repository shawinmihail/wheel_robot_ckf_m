function [X, sqrtP] = ekf_imu_calib_correction2(X, sqrtP, Z, sqrtR, a_mes)
n = length(X);
m = length(Z);

%% X [s e q]
g = [0;0;-10];
s = X(1:3);
e = X(4:6);
q = X(7:10);


%% mes model
% Z [dv_gnns/dt 0 0]
Z_x = quatRotate(q, diag(s)^-1 * a_mes - e) + g;
dz = Z - Z_x
dz2 = Z - (quatRotate([0.9950; -0.0995; 0; 0], diag(s)^-1 * a_mes - e) + g)

%% H
Zs = -quat2matrix(q) * diag(a_mes ./ s .^ 2);
Ze = -quat2matrix(q) * eye(3,3);
v = diag(s)^-1 * a_mes - e;
Zq = quat_rot_fcn_j(q(1),q(2),q(3),q(4),v(1),v(2),v(3));
H = [Zs Ze Zq];

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;
X(7:10) = X(7:10) / norm(X(7:10));
end

