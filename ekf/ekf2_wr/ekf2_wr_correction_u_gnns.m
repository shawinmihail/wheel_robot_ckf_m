function [X, sqrtP] = ekf2_wr_correction_u_gnns(X, sqrtP, Z, sqrtR)
n = length(X);
m = length(Z);

%% X [r v a q w]
v = X(4:6);
q = X(7:10);

%% mes model
% Z [vgnns]

ex = [1;0;0];
Z_x = quatRotate(q, ex * norm(v));
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

u = [norm(v); 0; 0];
Zuq = quat_rot_fcn_j(q(1),q(2),q(3),q(4), u(1),u(2),u(3));
Zuv = [norm_fcn_j(v(1),v(2),v(3)); zeros(2,3)];

H = [O33 Zuv Zuq];

%% square-root
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;

end

