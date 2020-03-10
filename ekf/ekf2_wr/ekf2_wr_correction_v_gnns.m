function [X, sqrtP] = ekf2_wr_correction_v_gnns(X, sqrtP, Z, sqrtR)

n = length(X);
m = length(Z);

%% X [r v q]
v = X(4:6);

%% mes model
% Z [vgnns]

Z_x = v;
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

H = [O33 E33 O34];

%% square-root
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;
end

