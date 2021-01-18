clc
clear 
close 


syms alpha z1 z2 z1s z2s mu Pi Pidot psi k v u
assume([alpha z1 z2 z1s z2s mu Pi Pidot psi k v u], 'real')

% alpha = 0;
P = [1 alpha; alpha alpha^2];
% z2 = sin(psi);
z = [z1; z2];
z1s = z2;
z2s = cos(psi)*(u-(k*cos(psi)/(z1*k+1)));
zs = [z1s; z2s];
V = z' * P * z + Pi;
Vdot = 2 * v * z' * P * zs + Pidot;
eq = Vdot + 2*mu*V
s = solve(eq, u);
ss = simplify(s)
% pretty(ss)
% lstr = latex(ss)
ret

% matlabFunction(ss,'file','obstacles_potential/u_obs_pot_generated.m')
retdr_obs = r - r_obs;
Pi = 0.5*(r_min - dr_obs)^2;
dr_obs_n = dr_obs / norm(dr_obs); % check on 0
dr_obs_dot = dot(v * [cos(theta); sin(theta)], dr_obs_n); 
Pidot = (r_min - dr_obs)*dr_obs_dot;

dr_obs = r - r_obs;
Pi = 0.5*(r_min - dr_obs)^2;
dr_obs_n = dr_obs / norm(dr_obs); % check on 0
dr_obs_dot = dot(v * [cos(theta); sin(theta)], dr_obs_n); 
Pidot = (r_min - dr_obs)*dr_obs_dot;