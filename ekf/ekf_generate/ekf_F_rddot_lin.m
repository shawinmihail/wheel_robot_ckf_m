clc
clear

q = sym('q', [4, 1]);
w = sym('w', [3, 1]);
a = sym('a', [3, 1]);
r = sym('r', [3, 1]);

assume([q; w; a; r], 'real')

rddot_q(q) = quatRotate(q, a + 0*cross(w,cross(r,w)))
rddot_w(w) = quatRotate(q, a + 0*cross(w,cross(r,w)))

rddot_q_j = jacobian(rddot_q, q)
rddot_w_j = jacobian(rddot_w, w)

qw = q(1);
qv = [q(2);q(3);q(4)];
f = [2*qw * a + cross(w,a)]

% matlabFunction(rddot_q_j,'file','ekf/ekf_routins/M_rddot_dq_fcn.m')
% matlabFunction(rddot_w_j,'file','ekf/ekf_routins/M_rddot_dw_fcn.m')
