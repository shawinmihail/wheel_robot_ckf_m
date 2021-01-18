clc
clear 
close 

syms alpha z1 z2 z1s z2s mu Pi Pis Piss psi k v u
assume([alpha z1 z2 z1s z2s mu Pi Pis Piss psi k v u], 'real')

%______________________________
syms ro r ev evx ndr theta sinThetaSq rmin
ro = sym('ro', [2 1])
r = sym('r', [2 1])
ev = sym('ev', [2 1])
evx = sym('evx', [1 2])
assume([ndr evx theta sinThetaSq rmin], 'real')
assume([r;ev], 'real')

dr = (ro-r);
drs = -ev;
ndrs = -ev' * dr / ndr;
Pi1 = 0.5 * (rmin - ndr)^2;
Pi2 = (ev'*dr + 1) / 2;
Pi = Pi1 * Pi2;
Pi1s = -(rmin - ndr)*ndrs;
Pi2s = u*evx*dr + ev'*drs;
Pis = Pi1s * Pi2 + Pi1 * Pi2s;
%___________________________________

z1s = z2;
z2s = cos(psi)*(u-(k*cos(psi)/(z1*k+1)));

pim = 0;
V = abs(z1) + abs(z2+k) + pim*Pi;
Vs = z1/abs(z1)*z2 + (z2+k)/abs(z2+k)*z2s + pim*Pis;

vim = 1;
eq = vim*Vs + vim*2*mu*V;
s = solve(eq, u);
ss = simplify(s)
pretty(ss)
% lstr = latex(ss)

matlabFunction(ss,'file','obstacles_potential/u_obs_pot_generated4_off.m')
ret


% retdr_obs = r - r_obs;
% Pi = 0.5*(r_min - dr_obs)^2;
% dr_obs_n = dr_obs / norm(dr_obs); % check on 0
% dr_obs_dot = dot(v * [cos(theta); sin(theta)], dr_obs_n); 
% Pidot = (r_min - dr_obs)*dr_obs_dot;
% 
% dr_obs = r - r_obs;
% Pi = 0.5*(r_min - dr_obs)^2;
% dr_obs_n = dr_obs / norm(dr_obs); % check on 0
% dr_obs_dot = dot(v * [cos(theta); sin(theta)], dr_obs_n); 
% Pidot = (r_min - dr_obs)*dr_obs_dot;