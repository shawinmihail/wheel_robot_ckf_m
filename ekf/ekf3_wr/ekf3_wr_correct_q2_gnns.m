function [X, sqrtP] = ekf3_wr_correct_q2_gnns(X, sqrtP, Z, R, dr1, dr2)

n = length(X);
m = length(Z);

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [dp1 dp2 dp3]
% dp1 = dr_slave1
% dp2 = dr_slave2

Z_dp1 = quatRotate(q, dr1);
Z_dp2 = quatRotate(q, dr2);
Z_x = [Z_dp1; Z_dp2];
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zq1 = quat_rot_fcn_j(q(1),q(2),q(3),q(4), dr1(1),dr1(2),dr1(3));
Zq2 = quat_rot_fcn_j(q(1),q(2),q(3),q(4), dr2(1),dr2(2),dr2(3));
H = [O33 O33 O33 Zq1 O33;
     O33 O33 O33 Zq2 O33];
 
 
%% ordinary K, H
P = sqrtP * sqrtP';
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
sqrtP = chol(P,'lower');
X = X + K*dz;
X(10:13) = X(10:13) / norm(X(10:13));
end

