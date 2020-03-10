function [X, sqrtP] = ekf2_wr_correction_a_imu(X, sqrtP, Z, sqrtR)
n = length(X);
m = length(Z);

%% X [r v q]
q = X(7:10);

%% mes model
% Z [aimu]

g = [0;0;-10];
Z_x = quatRotate(quatDual(q), [0;0;0] - g);
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zaq = quat_rot_fcn_j(q(1),-q(2),-q(3),-q(4),-g(1),-g(2),-g(3));
Zaq(:, 2:4) = -Zaq(:, 2:4);
H = [O33 O33 Zaq];

%% square-root
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;

end

