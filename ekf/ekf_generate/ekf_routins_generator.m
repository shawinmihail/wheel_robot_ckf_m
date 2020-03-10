clc
clear

q = sym('q', [4, 1]);
w = sym('w', [3, 1]);
v = sym('v', [3, 1]);
r = sym('r', [3, 1]);
dr = sym('dr', [3, 1]);
assume([q; w; v; r; dr], 'real')

%% quat rot lin
% quat_rot_fcn(q) = quatRotate(q, v);
% quat_rot_fcn_j = jacobian(quat_rot_fcn, q);
% matlabFunction(quat_rot_fcn_j,'file','ekf/ekf_routins/quat_rot_fcn_j.m')

%% norm lin
norm_fcn(v) = norm(v);
norm_fcn_j = jacobian(norm_fcn, v);
matlabFunction(norm_fcn_j,'file','ekf/ekf_routins/norm_fcn_j.m')