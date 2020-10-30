function [X, sqrtP] = ekf3_wr_corrrect_w_imu(X, sqrtP, Z, R, g)

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [a_imu];
Z_x = w;
dz = Z - Z_x;


%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

H = [O33 O33 O33 O34 E33];
 
%% ordinary K, H
P = sqrtP * sqrtP';
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
sqrtP = chol(P,'lower');
X = X + K*dz;
X(10:13) = X(10:13) / norm(X(10:13));
end

