function [X, sqrtP] = ekf_wr_correction_v_abs_and_dir_gnns(X, sqrtP, Z, sqrtR, gps_attachment_r)
n = length(X);
m = length(Z);

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [vgnns]
% vgnns = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w, dr));
ex = [1;0;0];
Z_x = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w, gps_attachment_r));
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zvq = Z_vgnns_ad_dq_fcn(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),v(1),v(2),v(3),w(1),w(2),w(3));
Zvv = Z_vgnns_ad_dv_fcn(v(1),v(2),v(3),q(1),q(2),q(3),q(4));
Zvw = Z_vgnns_dw_fcn(gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),q(1),q(2),q(3),q(4));
H = [O33 Zvv O33 Zvq Zvw];

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;

end

