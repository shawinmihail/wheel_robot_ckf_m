clc
clear 
close 

% define symbolic
syms  z1 z2 z1s z1ss lambda v u phi delta Delta norm_ps Delta_pss pse
assume([z1 z2 z1s z1ss lambda v u phi delta Delta norm_ps Delta_pss pse], 'real');

% define model
z1s = cos(phi);
z1ss = 1 / delta * (1 - cos(phi)^2 - (norm_ps^2 * sin(phi)^2 / (norm_ps^2 - Delta_pss))) + u * sin(phi);     

% resolve u
eq = z1ss + 2*lambda*z1s + lambda*lambda*delta;
s = solve(eq, u);
ss = simplify(s)
