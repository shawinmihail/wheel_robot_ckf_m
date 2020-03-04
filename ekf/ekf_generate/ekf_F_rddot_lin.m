clc
clear

q = sym('q', [4, 1]);
w = sym('w', [3, 1]);
a = sym('a', [3, 1]);
r = sym('r', [3, 1]);

assume([q; w; a; r], 'real')
r = zeros(3,1); %% zero now

rddot_q(q) = quatRotate(q, a + cross(w,cross(r,w)))
rddot_w(w) = quatRotate(q, a + cross(w,cross(r,w)))

rddot_q_j = jacobian(rddot_q, q)
rddot_w_j = jacobian(rddot_w, w)

% matlabFunction(rddot_q_j,'file','ekf/ekf_routins/M_rddot_dq_fcn.m')
% matlabFunction(rddot_w_j,'file','ekf/ekf_routins/M_rddot_dw_fcn.m')
