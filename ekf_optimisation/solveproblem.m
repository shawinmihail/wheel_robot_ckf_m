clc
close all

%% init
c1 = clock

% %--------------------------
% load('ekf_optimisation/out/x')
% x0 = x;
% %---------
a_gnns_x = -0;
a_gnns_z = -0;
r_gnns_x = -2;
r_gnns_z = -2;
v_gnns_x = -4;
v_gnns_z = -4;
u_gnns_x = -4;
u_gnns_z = -2;
q_gnns_x = -5;
q_gnns_z = -5;
%--------------
qr = -1;
qv = -1;
qqx = -4;
qqz = -4;
%--------------
x0 = [a_gnns_x a_gnns_z r_gnns_x r_gnns_z v_gnns_x v_gnns_z u_gnns_x u_gnns_z q_gnns_x q_gnns_z...
      qr qv qqx qqz];
% ---------------
problem_x = @(x) problem(x);
options = optimset('MaxIter', 500, 'TolFun', 1e-4, 'TolX', 0.25, 'PlotFcns',@optimplotfval);
[x, res] = fminsearch(problem_x, x0, options);

%% postproc
c2 = clock;
dc = (c2 - c1)'

save('ekf_optimisation/out/x', 'x')
