function [X, P] = ekf4_wr_correct_u(X, P, Z, R, w_imu, dr_bshc_gnns)

%% X [r v q]
r = X(1:3);
v = X(4:6);
q = X(7:10);

%% mes model
% Z [vgnns]
% vgnns = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w, dr));
% vb = quatRotate(quatDual(q), v);
% if norm(vb) < 0.15
%     return
% end
ex = [1;0;0];
Z_x = quatRotate(q, ex * norm(v)) + quatRotate(q, cross(w_imu, dr_bshc_gnns));
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zvq = Z_vgnns_ad_dq_fcn(q(1),q(2),q(3),q(4),dr_bshc_gnns(1),dr_bshc_gnns(2),dr_bshc_gnns(3),v(1),v(2),v(3),w_imu(1),w_imu(2),w_imu(3));
% vv = ex * norm(v);
% Zvq = quat_rot_fcn_j(q(1),q(2),q(3),q(4),vv(1),vv(2),vv(3));
Zvv = Z_vgnns_ad_dv_fcn(v(1),v(2),v(3),q(1),q(2),q(3),q(4));
Zvq = Zvq(:,2:4); % correct only vector part
H = [O33 Zvv Zvq];

%% ordinary K, H
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
dx = K*dz;
dx = [dx(1:6); 0; dx(7:9)];
X = X + dx;
X(7:10) = X(7:10) / norm(X(7:10));
end

