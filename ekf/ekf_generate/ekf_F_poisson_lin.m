clc
clear

q = sym('q', [4, 1]);
w = sym('w', [3, 1]);
assume([q; w], 'real')
qw = [0; w];

qdot_q(q) = 0.5*quatMultiply(q , qw);
qdot_w(w) = 0.5*quatMultiply(q , qw);

qdot_q_j = jacobian(qdot_q, q)
qdot_w_j = jacobian(qdot_w, w)

matlabFunction(qdot_q_j,'file','ekf/ekf_routins/M_qdot_dq_fcn.m')
matlabFunction(qdot_w_j,'file','ekf/ekf_routins/M_qdot_dw_fcn.m')

