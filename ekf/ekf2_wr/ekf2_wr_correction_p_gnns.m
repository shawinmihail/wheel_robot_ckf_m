function [X, sqrtP] = ekf2_wr_correction_p_gnns(X, sqrtP, Z, sqrtR, gps_attachment_r)

n = length(X);
m = length(Z);

%% X [r v a q]
r = X(1:3);
q = X(10:13);

%% mes model
% Z [rgnns]

Z_x = r + quatRotate(q, gps_attachment_r);
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zrq = quat_rot_fcn_j(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3));
H = [E33 O33 O33 Zrq];

%% corr
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;

end

