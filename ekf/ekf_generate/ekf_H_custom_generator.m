clc
clear


% Z_v = quatRotate(q, ex * norm(v));
% Z_a= quatRotate(quatDual(q), a - g);

q = sym('q', [4, 1]);
a = sym('a', [3, 1]);
v = sym('v', [3, 1]);
g = sym('g', [3, 1]);

assume([q; a; g], 'real')

%%
vgnns_q(q) = quatRotate(q, [norm(v);0;0]);
vgnns_v(v) = quatRotate(q, [norm(v);0;0]);

vgnns_q_j = jacobian(vgnns_q, q);
vgnns_v_j = jacobian(vgnns_v, v);

matlabFunction(vgnns_q_j,'file','ekf/ekf_routins/Zvq_fcn.m')
matlabFunction(vgnns_v_j,'file','ekf/ekf_routins/Zvv_fcn.m')


%% 
imu_a_q(q) = quatRotate(quatDual(q), a - g);
imu_a_q_j = jacobian(imu_a_q, q);
matlabFunction(imu_a_q_j,'file','ekf/ekf_routins/Zaq_fcn.m')



