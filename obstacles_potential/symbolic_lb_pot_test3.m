clc
clear 
close 

syms alpha z1 z2 z1s z2s mu Pi Pis Piss psi k v u
assume([alpha z1 z2 z1s z2s mu Pi Pis Piss psi k v u], 'real')

% ______________________________
% syms ro r ev evx ndr theta Theta rmin
% assume([ro r ev evx ndr theta Theta rmin], 'real')
% 
% dr = ro-r;
% drs = - ev;
% ndrs = -ev' * dr / ndr;
% ndrss = (-u*evx*dr+sin(Theta)^2)/ndr;
% Pi = 0.5 * (rmin - ndr)^2;
% Pis = -(rmin - ndr)*ndrs;
% Piss = ndrs^2 - (rmin - ndr)*ndrss;
%___________________________________

z1s = z2;
z2s = cos(psi)*(u-(k*cos(psi)/(z1*k+1)));

V = abs(z1) + abs(z2) + Pi;
Vs = z1/abs(z1)*z2 + (z2)/abs(z2+1)*z2s + Pis;

eq = Vs + 2*mu*V;
s = solve(eq, u);
ss = simplify(s)
pretty(ss)
% lstr = latex(ss)
ret
matlabFunction(ss,'file','obstacles_potential/u_obs_pot_generated3.m')
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