function [X, sqrtP, sqrtR] = ekf_wr_correction_pv_gnns(X, sqrtP, Z, sqrtR)
% X [r v q]
% Z [r v]

%% mes model
%% P
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

H = [E33 O33 O34;
     O33 E33 O34];
%% mes error
dz = Z - H * X;

%% ordinary K, H
P = sqrtP * sqrtP';
R = sqrtR * sqrtR';
R = R + H*P*H';
K = P * H' * R^-1;
P = P - K * H * P;
sqrtP = chol(P,'lower');
% sqrtR = chol(R,'lower');

% square-root K, H
% M = tria([sqrtR, H * sqrtP; zeros(10, 6), sqrtP], 16);
% sqrtR = M(1:6, 1:6);
% K = M(7:16, 1:6);
% sqrtP = M(7:16, 7:16);

%% state est
X = X + K*dz;


end

