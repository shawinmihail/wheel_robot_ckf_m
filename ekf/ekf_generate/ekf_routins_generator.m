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
% norm_fcn(v) = norm(v);
% norm_fcn_j = jacobian(norm_fcn, v);
% matlabFunction(norm_fcn_j,'file','ekf/ekf_routins/norm_fcn_j.m')

%% cross quatrotated
% f(v) = cross(quatRotate(q, w), v);
% f_j_q = jacobian(f, q)
% f_j_w = jacobian(f, w)
% 
% % cross(quatRotate(q, w), v) = -cross_operator(v) * quatRotate(q, w) =
% % -cross_operator(v) R(q) w
% 
% f_j_q1 = -x_operator(v) * quat_rot_fcn_j(q(1),q(2),q(3),q(4),w(1),w(2),w(3));
% f_j_w1 = -x_operator(v) * quat2matrix(q);
% 
% rng(200)
% v_ = randn(3,1);
% w_ = randn(3,1);
% q_ = randn(4,1);
% q_ = q_ / norm(q_);
% 
% vpa_f_j_q = vpa(subs(f_j_q, [v; w; q], [v_; w_; q_]))
% vpa_f_j_q1 = vpa(subs(f_j_q1, [v; w; q], [v_; w_; q_]))
% 
% vpa_f_j_w = vpa(subs(f_j_w, [v; w; q], [v_; w_; q_]))
% vpa_f_j_w1 = vpa(subs(f_j_w1, [v; w; q], [v_; w_; q_]))

%% quatrotate double cross
f = quatRotate(q, cross(w, cross(dr, w)))
g = cross(w, cross(dr, w));
g_j_w = jacobian(g, w)
g_j_w_1 = x_operator(w)*x_operator(dr) - x_operator(cross(dr, w))


% f = w x (dr x w)
% f = hatW (dr x w)
% df = d(hatW) (dr x w) + hatW hatR


%%
% matlabFunction(norm_fcn_j,'file','ekf/ekf_routins/norm_fcn_j.m')