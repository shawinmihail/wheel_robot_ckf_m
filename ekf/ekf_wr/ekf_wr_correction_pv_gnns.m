function [X, sqrtP] = ekf_wr_correction_pv_gnns(X, sqrtP, Z, sqrtR, gps_attachment_r)

n = length(X);
m = length(Z);

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [rgnns vgnns]
% rgnns = r + quatRotate(q, dr);
% vgnns = v + quatRotate(q, cross(w, dr));

Z_r = r + quatRotate(q, gps_attachment_r);
Z_v = v + quatRotate(q, cross(w, gps_attachment_r));
Z_x = [Z_r; Z_v];
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zrq = Z_rgnns_dq_fcn(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3));
Zvq = Z_vgnns_dq_fcn(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),w(1),w(2),w(3));
Zvw = Z_vgnns_dw_fcn(w(1),w(2),w(3),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),q(1),q(2),q(3),q(4));

% pos vel stright corrections pos = pos_mes, vel = vel_mes
H = [E33 O33 O33 Zrq O33;
     O33 E33 O33 Zvq Zvw];

%% ordinary K, H
% P = sqrtP * sqrtP';
% R = sqrtR * sqrtR';
% Rk = R + H*P*H';
% K = P * H' * (Rk)^-1;
% P = P - K *H *P;
% sqrtP = chol(P,'lower');
% X1 = X + K*dz;

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;
end

