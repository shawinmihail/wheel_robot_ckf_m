function [X, sqrtP] = ekf_wr_correction_g_imu(X, sqrtP, Z, sqrtR)
% X [r v qv]
% Z [a_imu]

n = length(X);
m = length(Z);

%% mes model
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

% if robot velocity is constant
% a_imu = Q(quatDual(q)) * -g;
g = [0;0;-10];
q = X(7:10);
qd = quatDual(q);
% a_imu = quatRotate(quatDual(q), -g);
Maq = quat_dual_rot_jacob(q(1),q(2),q(3),q(4),-g(1),-g(2),-g(3));
H = [O33 O33 Maq];
%% mes error
dz = Z - quatRotate(qd, -g);

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
% d = K*(sqrtRk')^-1*dz
X = X + K * (sqrtRk')^-1 * dz;

% q = X(7:10);
% qd = quatDual(q);
% nq2 = norm(q)
% dz2 = Z - quatRotate(qd, -g)

end

