% clc
% clear
% close all
% 
% % p = sym('p', [3, 1])
% % w = sym('w', [3, 1])
% % d = sym('d', [3, 1])
% % e = sym('e', [3, 1])
% % assume(p, 'real')
% % assume(e, 'real')
% % assume(d, 'real')
% % assume(w, 'real')
% % 
% % 
% % a =  d' * cross(w, e)
% % b = cross(e, d)' * w
% % b = collect(b, d)
% % simplify(a-b)
% 
% % b = collect(a, w(3))
% % 
% % c = d(1)*e(3)*w(2) - d(3)*(e(1)*w(2) - e(2)*w(1)) - d(2)*e(3)*w(1)
% % d = (d(2)*e(1) - d(1)*e(2))
% % simplify(c/d)
% 
% syms x lambda z1(x) z2(x)
% 
% % eqs = [diff(z1,x) == z2; diff(z2,x) == -2 * lambda * z2 - lambda^2*z1]
% % cond = [z1(0) == 1; z2(0) == 0];
% % S = dsolve(eqs, cond)
% % S.z1
% % S.z2
% 
% % eqs = [diff(z1,x) == z2/z1; diff(z2,x) == z2*z1]
% % S = dsolve(eqs)
% % pretty(S.z1)
% % pretty(S.z2)
% 
% eqs = [diff(z1,x) == -1/z1]
% S = dsolve(eqs)
% pretty(S)
% 
% % tspan = [0 5];
% % z0 = [1, 0.5];
% % f = @(t, z)[z(2)/z(1); -z(2)];
% % [t,z] = ode45(f, tspan, z0);
% % z
% % figure
% % hold on
% % plot(t,z(:,1),'r')
% % plot(t,z(:,2),'g')

clc
clear
close all
n = 50;
d = 1;
x = -n: d : n;
[X, Y] = meshgrid(x, x);
% Z = 0.05 * (X .^2 - Y .^ 2);
Z = 2*sin(X*2*pi / 25) +  3*sin(Y*2*pi /35);
name = ['sin_plus_sin_' num2str(n) '_' num2str(d) '.stl']
surf2stl(name, X, Y, Z);

