clc
clear 
close 

e = sym('e', [3 1]);
w = sym('w', [3 1]);
d = sym('d', [3 1]);
assume([e;w;d], 'real');

d' * x_operator(w) * e
cross(e,d)' * w
% x_operator(e)' * w
% -x_operator(e) * w
ret

% define symbolic
syms alpha z1 z2 z1s z2s mu Pi Pis Piss psi k v u
assume([alpha z1 z2 z1s z2s mu Pi Pis Piss psi k v u], 'real');

syms ro r ev evx ndr theta sinThetaSq rmin
ro = sym('ro', [2 1]);
r = sym('r', [2 1]);
ev = sym('ev', [2 1]);
evx = sym('evx', [1 2]);
assume([ndr evx theta sinThetaSq rmin], 'real')
assume([r;ev], 'real')

% define potential
dr = ro-r;
drs = -ev;
ndrs = -ev' * dr / ndr;
ndrss = (-u*evx*dr+sinThetaSq)/ndr;
Pi = 0.5 * (rmin - ndr)^2;
Pis = -(rmin - ndr)*ndrs;
Piss = ndrs^2 - (rmin - ndr)*ndrss;
%___________________________________

% define model
z1s = z2;
z2s = cos(psi)*(u-(k*cos(psi)/(z1*k+1)));

% define potential
syms Pi_mult % potential multiplicator
V = abs(z1) + Pi_mult*Pi;
Vs = z1/abs(z1)* z2 + Pi_mult*Pis;
Vss = z1/abs(z1)* z2s + Pi_mult*Piss;

% resolve u
eq = Vss + 2*mu*Vs + mu*mu*V;
s = solve(eq, u);
ss = simplify(s)
% pretty(ss)
% lstr = latex(ss)

% generate matlab function
matlabFunction(ss,'file','obstacles_potential/u_obs_pot_generated_example.m')