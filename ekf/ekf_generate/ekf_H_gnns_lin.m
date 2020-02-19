clc
clear

q = sym('q', [4, 1]);
w = sym('w', [3, 1]);
v = sym('v', [3, 1]);
r = sym('r', [3, 1]);
dr = sym('dr', [3, 1]);
assume([q; w; v; r; dr], 'real')

%% rv
% rgnns_q(q) = quatRotate(q, dr);
% vgnns_q(q) = quatRotate(q, cross(w, dr));
% vgnns_w(w) = quatRotate(q, cross(w, dr));
% 
% rgnns_q_j = jacobian(rgnns_q, q)
% vgnns_q_j = jacobian(vgnns_q, q)
% vgnns_w_j = jacobian(vgnns_w, w)
% 
% matlabFunction(rgnns_q_j,'file','ekf/ekf_routins/Z_rgnns_dq_fcn.m')
% matlabFunction(vgnns_q_j,'file','ekf/ekf_routins/Z_vgnns_dq_fcn.m')
% matlabFunction(vgnns_w_j,'file','ekf/ekf_routins/Z_vgnns_dw_fcn.m')

%% v_abs_and_dir
vgnns_q(q) = quatRotate(q, [norm(v);0;0]) + quatRotate(q, cross(w, dr));
vgnns_v(v) = quatRotate(q, [norm(v);0;0]) + quatRotate(q, cross(w, dr));

vgnns_q_j = jacobian(vgnns_q, q);
vgnns_v_j = jacobian(vgnns_v, v);

matlabFunction(vgnns_q_j,'file','ekf/ekf_routins/Z_vgnns_ad_dq_fcn.m')
matlabFunction(vgnns_v_j,'file','ekf/ekf_routins/Z_vgnns_ad_dv_fcn.m')
