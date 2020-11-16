function [X, P] = ekf4_wr_correct_q2_gnns(X, P, Z, R, dr1, dr2)

%% X [r v q]
r = X(1:3);
v = X(4:6);
q = X(7:10);

%% mes model
% Z [dp1 dp2]
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
Zq1 = Zq1(:,2:4); % correct only vector part
Zq2 = Zq2(:,2:4); % correct only vector part
H = [O33 O33 Zq1;
     O33 O33 Zq2];
 
 
%% ordinary K, H
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
dx = K*dz;
dx = [dx(1:6); 0; dx(7:9)];
X = X + dx;
X(7:10) = X(7:10) / norm(X(7:10));
end

