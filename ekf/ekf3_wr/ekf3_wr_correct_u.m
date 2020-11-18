function [X, sqrtP] = ekf3_wr_correct_u(X, sqrtP, Z, R, dr_bshc_gnns)

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [vgnns]
% vgnns = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w, dr));
% vb = quatRotate(quatDual(q), v);
% if norm(vb) < 0.15
%     return
% end
ex = [1;0;0];
Z_x = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w, dr_bshc_gnns));
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zvq = Z_vgnns_ad_dq_fcn(q(1),q(2),q(3),q(4),dr_bshc_gnns(1),dr_bshc_gnns(2),dr_bshc_gnns(3),v(1),v(2),v(3),w(1),w(2),w(3));
Zvv = Z_vgnns_ad_dv_fcn(v(1),v(2),v(3),q(1),q(2),q(3),q(4));
Zvw = Z_vgnns_dw_fcn(dr_bshc_gnns(1),dr_bshc_gnns(2),dr_bshc_gnns(3),q(1),q(2),q(3),q(4));
H = [O33 Zvv O33 Zvq Zvw];

%% ordinary K, H
P = sqrtP * sqrtP';
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
sqrtP = chol(P,'lower');
X = X + K*dz;
X(10:13) = X(10:13) / norm(X(10:13));
end

