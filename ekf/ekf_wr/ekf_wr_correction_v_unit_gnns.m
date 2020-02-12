function [X, sqrtP] = ekf_wr_correction_v_unit_gnns(X, sqrtP, Z, sqrtR)
% X [r v q]
% Z [v_unit]

n = length(X);
m = length(Z);

%% mes model
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

%% v_unit = quatRotate(q, [1;0;0])
%% v_unit = v/norm(v) -- can be added to H
q = X(7:10);
Mvq = quat_rot_jacob(q(1),q(2),q(3),q(4), 1, 0, 0);
H = [O33 O33 Mvq];
%% mes error
dz = Z - quatRotate(q, [1;0;0]);


%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;

end

