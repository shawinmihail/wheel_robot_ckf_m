clc
clear

q = sym('q', [4, 1]);
s = sym('s', [3, 1]);
e = sym('e', [3, 1]);
a = sym('a', [3, 1]);
assume([q; s; e; a], 'real')

-diag(a ./ s .^ 2)
f(s) = diag(s)^-1 * a;
f_j = jacobian(f, s)

% Z_x = quatRotate(q, diag(s)^-1 * aMes - e) + g;
% 
% %% norm lin
% norm_fcn(v) = norm(v);
% norm_fcn_j = jacobian(norm_fcn, v);
% matlabFunction(norm_fcn_j,'file','ekf/ekf_routins/norm_fcn_j.m')