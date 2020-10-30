clc
close all

%% init
c1 = clock

R_a_imu = diag(10e-2*[1; 1; 1]);
R_a_gnns = diag(5e-2*[1; 1; 1]);

R_w_imu = diag(1e-8*[1; 1; 1]);

R_rv_gnns = diag([1e-2*[1; 1; 1]; 1e-2*[1; 1; 1]]);

R_u_gnns = diag(1e-4*[1; 1; 1]);
R_q2_gnns = diag(1e-8*[1; 1; 1; 1; 1; 1]);

%--------------------------
load('ekf_optimisation/out/x')
x0 = x
% %---------
% a_imu_xz = -1;
% a_gnns_x = log(5e-2)/log(10);
% a_gnns_z = log(5e-2)/log(10);
% w_imu_xz = -8;
% r_gnns_x = -2;
% r_gnns_z = -2;
% v_gnns_x = -2;
% v_gnns_z = -2;
% u_gnns_x = -4;
% u_gnns_z = -4;
% %--------------
% qr = -1;
% qv = -1;
% qax = log(25e-2)/log(10);
% qaz = log(25e-2)/log(10);
% qq0 = -3;
% qqx = -3;
% qqz = -3;
% qwx = -2;
% qwz = log(15e-2)/log(10);
% %--------------
% x0 = [a_imu_xz a_gnns_x a_gnns_z w_imu_xz r_gnns_x r_gnns_z v_gnns_x v_gnns_z u_gnns_x u_gnns_z ...
%     qr qv qax qaz qq0 qqx qqz qwx qwz];
% %---------------
problem_x = @(x) problem(x);
options = optimset('MaxIter', 500, 'TolFun', 1e-3, 'TolX', 0.1, 'PlotFcns',@optimplotfval);
[x, res] = fminsearch(problem_x, x0, options);

%% postproc
c2 = clock;
dc = (c2 - c1)'

save('ekf_optimisation/out/x', 'x')
% save('reses/res', 'res')
% saveas(gcf,['report2/long_step/' num2str(timestep) 'min.png'])
% saveas(gcf,['report2/long_step/' num2str(timestep) 'min.fig'])