function [X, sqrtP] = ekf3_wr_corrrect_a_imu(X, sqrtP, Z, R, g)

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [a_imu];
Z_x = quatRotate(quatDual(q), a - g);
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Z_aimu_a = Z_aimu_da_fcn(q(1),q(2),q(3),q(4));
Z_aimu_q = Z_aimu_dq_fcn(q(1),-q(2),-q(3),-q(4),a(1),a(2),a(3),g(1),g(2),g(3));
Z_aimu_q(:, 2:4) = -Z_aimu_q(:, 2:4);
Z_aimu_q(:,4) = Z_aimu_q(:,4)*1; % assume we do not correct yaw with imu acc !!!!

H = [O33 O33  Z_aimu_a  Z_aimu_q    O33];
 
%% ordinary K, H
P = sqrtP * sqrtP';
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
sqrtP = chol(P,'lower');
X = X + K*dz; % mb we can remove from K angle vel parts
X(10:13) = X(10:13) / norm(X(10:13));
end

