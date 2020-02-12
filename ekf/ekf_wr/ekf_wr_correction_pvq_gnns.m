function [X, sqrtP] = ekf_wr_correction_pvq_gnns(X, sqrtP, Z, sqrtR)
% X [r v q]
% Z [r v q]

n = length(X);
m = length(Z);

%% mes model
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);
E44 = eye(4, 4);

H = [E33 O33 O34;
     O33 E33 O34;
     O43 O43 E44];
%% mes error
dz = Z - X;

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;

end

