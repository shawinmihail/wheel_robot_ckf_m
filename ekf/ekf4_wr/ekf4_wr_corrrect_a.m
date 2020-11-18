function [X, P] = ekf4_wr_corrrect_a(X, P, Z, a_imu, w_imu, R, dr_imu_gnns, g)

%% X [r v a q w]
r = X(1:3);
v = X(4:6);
q = X(7:10);


%% mes model
% Z [a_gnns];
% a_gnns = a - dr_gnns * w_dot - w cross (dr_gnss cross w);
% aI = quatRotate(q, a_imu) - g
% assume mean w_dot is about 0
Z_gnns = quatRotate(q, a_imu) - g + quatRotate(q, cross(w_imu, cross(dr_imu_gnns, w_imu)));
dz = Z - Z_gnns;



%% H
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
O44 = zeros(4, 4);
E33 = eye(3, 3);

vv = a_imu;
Z_aimu_q = quat_rot_fcn_j(q(1),q(2),q(3),q(4),vv(1),vv(2),vv(3));
% Z_aimu_q(:,4) = Z_aimu_q(:,4)*0; % assume we do not correct yaw with imu acc !!!!
vv = cross(w_imu, cross(dr_imu_gnns, w_imu));
Z_wimu_q = quat_rot_fcn_j(q(1),q(2),q(3),q(4),vv(1),vv(2),vv(3));

Zq = Z_aimu_q + Z_wimu_q;
Zq = Zq(:, 2:4); % correct only vector part

H = [O33 O33 Zq];
 
%% ordinary K, H
Rk = R + H*P*H';
K = P * H' * (Rk)^-1;
P = P - K *H *P;
dx = K*dz;
dx = [dx(1:6); 0; dx(7:9)];
X = X + dx;
X(7:10) = X(7:10) / norm(X(7:10));

r = X(1:3);
v = X(4:6);
q = X(7:10);
Z_gnns = quatRotate(quatDual(q), a_imu - g) + quatRotate(q, cross(w_imu, cross(dr_imu_gnns, w_imu)));
end

