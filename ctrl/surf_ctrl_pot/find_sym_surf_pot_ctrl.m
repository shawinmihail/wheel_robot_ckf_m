clc
clear 
close 

% define symbolic
syms  z1 z2 abs_z1 z2s V Vs Vss mu u phi delta Delta norm_ps Delta_pss signX
assume([z1 z2 abs_z1 z2s V Vs Vss mu u phi delta Delta norm_ps Delta_pss signX], 'real');

% define model
z1 = delta;
z2 = cos(phi);
z2s = 1 / delta * (sin(phi)^2 - (norm_ps^2 * sin(phi)^2 / (norm_ps^2 - Delta_pss))) + u * sin(phi)*signX;

% define potential
syms ro r ev evx ndr theta v rmin
ro = sym('ro', [3 1]);
r = sym('r', [3 1]);
ev = sym('ev', [3 1]);
% evx = sym('evx', [1 3]);
assume([ndr evx theta v rmin], 'real')
assume([r;ev], 'real')

dr = ro-r;
drs = -ev;
ndrs = -v*cos(theta);
ndrss = v^2*u*sin(theta);
% ndrss = ev'*ev*u*sin(theta)/ndr;

% Pi = 0.5 * (rmin - ndr)^2;
% Pis = -(rmin - ndr)*ndrs;
% Piss = ndrs^2 - (rmin - ndr)*ndrss;

syms Pi Pis
assume([Pi Pis], 'real')

syms Pi_mult 
V = abs(z1) + Pi_mult*Pi;
Vs = z1/abs(z1)* z2 + Pi_mult*Pis;
Vss = z1/abs(z1)* z2s

eq = Vss + 2*mu*Vs + mu*mu*V;
s = solve(eq, u);
pretty(s)
ss = simplify(s)
% matlabFunction(ss,'file','ctrl/surf_ctrl_pot/surf_ctrl_pot_test_1.m')


