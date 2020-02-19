function [X, sqrtP] = ekf_wr_correction_custom(X, sqrtP, Z, sqrtR, gps_attachment_r)

n = length(X);
m = length(Z);

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
a = X(7:9);
q = X(10:13);
w = X(14:16);

%% mes model
% Z [rgnns vgnns a_imu]
% rgnns = r + quatRotate(q, dr);
g = [0;0;-10];
Z_r = r + quatRotate(q, gps_attachment_r);
Z_v = quatRotate(q, [1;0;0] * norm(v));
Z_a = quatRotate(quatDual(q), a - g);

Z_x = [Z_r; Z_v; Z_a];
dz = Z - Z_x;

%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

Zaq = Zaq_fcn(q(1),q(2),q(3),q(4),a(1),a(2),a(3),g(1),g(2),g(3));
Zvq = Zvq_fcn(q(1),q(2),q(3), q(4), v(1),v(2),v(3));
Zvv = Zvv_fcn(v(1),v(2),v(3),q(1),q(2),q(3), q(4));

H = [E33 O33 O33 O34 O33;
     O33 Zvv O33 Zvq O33;
     O33 O33 O33 Zaq O33];

%% square-root K, H
M = tria([sqrtR, H * sqrtP; zeros(n, m), sqrtP], m + n);
sqrtRk = M(1:m, 1:m);
K = M(m + 1:m + n, 1:m);
sqrtP = M(m + 1:n + m, m + 1:n + m);
X = X + K*(sqrtRk')^-1*dz;
end

