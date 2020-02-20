function [X, sqrtP] = ekf_wr_correction_custom(X, sqrtP, Z, sqrtR, gps_attachment_r, imu_attachment_r)

n = length(X);
m = length(Z);

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [rgnns vgnns a_imu]
% rgnns = r + quatRotate(q, dr);
g = [0;0;-10];
ex = [1;0;0];

Z_r = r + quatRotate(q, gps_attachment_r);
Z_v1 = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w, gps_attachment_r));
Z_v2 = v + quatRotate(q, cross(w, gps_attachment_r));
Z_a = quatRotate(quatDual(q), a - g) - cross(w, cross(imu_attachment_r, w));

Z_x = [Z_r; Z_v1; Z_v2; Z_a];
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zrq = Z_rgnns_dq_fcn(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3));

Zvq1 = Z_vgnns_ad_dq_fcn(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),v(1),v(2),v(3),w(1),w(2),w(3));
Zvv1 = Z_vgnns_ad_dv_fcn(v(1),v(2),v(3),q(1),q(2),q(3),q(4));
Zvw1 = Z_vgnns_dw_fcn(w(1),w(2),w(3),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),q(1),q(2),q(3),q(4));

Zvq2 = Z_vgnns_dq_fcn(q(1),q(2),q(3),q(4),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),w(1),w(2),w(3));
Zvv2 = E33;
Zvw2 = Z_vgnns_dw_fcn(w(1),w(2),w(3),gps_attachment_r(1),gps_attachment_r(2),gps_attachment_r(3),q(1),q(2),q(3),q(4));

Zaa = Z_aimu_da_fcn(a(1),a(2),a(3),q(1),q(2),q(3),q(4));
Zaq = Z_aimu_dq_fcn(q(1),q(2),q(3),q(4),a(1),a(2),a(3),g(1),g(2),g(3));
Zaw = Z_aimu_dw_fcn(w(1),w(2),w(3),imu_attachment_r(1),imu_attachment_r(2),imu_attachment_r(3));

H = [E33 O33  O33 Zrq O33;
     O33 Zvv1 O33 Zvq1 Zvw1;
     O33 Zvv2 O33 Zvq2 Zvw2;
     O33 O33  Zaa Zaq Zaw];
 
 %% ordinary K, P
P = sqrtP * sqrtP';
R = sqrtR * sqrtR';
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K * H *P;
sqrtP = chol(P,'lower');

%% square-root K, P
% M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
% sqrtRk = M(1:m, 1:m);
% K = M(m + 1:m + n, 1:m);
% sqrtP = M(m + 1:m + n, m + 1:m + n);
% dz = (sqrtRk')^-1 * dz;

%% corrections
X(10:13) = X(10:13) / norm(X(10:13));
X = X + K*dz;
end

