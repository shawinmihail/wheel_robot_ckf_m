clc
clear

q = sym('q', [4, 1]);
v = sym('v', [3, 1]);

Q = quat2matrix(quatDual(q));

f(q) = Q*v;
dfdq = jacobian(f, q)
matlabFunction(dfdq,'file','ekf/ekf_routins/quat_dual_rot_jacob.m')

% q = sym('q', [4, 1]);
% v = sym('v', [3, 1]);
% qv = [0; v];
% 
% f(q) = 0.5*quatMultiply(q , qv);
% dfdq = jacobian(f, q);
% matlabFunction(dfdq,'file','ekf/ekf_routins/poison_eq_jacob.m')

% q = [100 30 10 15]';
% q = q / norm(q);
% Q = quat2matrix(q);
% 
% dq = [1 2 3 2]' * 1e-2;
% Q1 = quat2matrix(q+dq);
% 
% v = [1 2 3]';
% 
% p = Q*v;
% p_dp = Q1 * v;
% dp1 = p_dp - p
% 
% dp2 = testfcn(q(1),q(2),q(3),q(4),v(1),v(2),v(3)) * dq
