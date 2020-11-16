function [X, P] = ekf4_wr_correct_rv_gnns(X, P, Z, R, w, dr_imu_gnns)

%% X [r v q]
r = X(1:3);
v = X(4:6);
q = X(7:10);

%% mes model
% Z [rgnns vgnns]
% rgnns = r + quatRotate(q, dr);
% vgnns = v + quatRotate(q, cross(w, dr));

Z_r = r + quatRotate(q, dr_imu_gnns);
Z_v = v + quatRotate(q, cross(w, dr_imu_gnns));
Z_x = [Z_r; Z_v];
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zrq = Z_rgnns_dq_fcn(q(1),q(2),q(3),q(4),dr_imu_gnns(1),dr_imu_gnns(2),dr_imu_gnns(3));
Zvq = Z_vgnns_dq_fcn(q(1),q(2),q(3),q(4),dr_imu_gnns(1),dr_imu_gnns(2),dr_imu_gnns(3),w(1),w(2),w(3));
Zrq = Zrq(:,2:4); % correct only vector part
Zvq = Zvq(:,2:4); % correct only vector part
H = [E33 O33  Zrq ;
     O33 E33  Zvq];
 

%% ordinary K, H
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
dx = K*dz;
dx = [dx(1:6); 0; dx(7:9)];
X = X + dx;
X(7:10) = X(7:10) / norm(X(7:10));
end

