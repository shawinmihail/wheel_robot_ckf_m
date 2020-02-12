clc
clear
close all

syms x1 y1 z1
syms x2 y2 z2
syms a b

syms f1(x, y, z)
syms f2(x, y, z)

%% f
f(x, y, z) = a * x^2 + b * y + z;
f1 = subs(f, [x y z], [x1 y1 z1])
f2 = subs(f, [x y z], [x2 y2 z2])

%% grad f
grad_f1 = [diff(f1, x1); diff(f1, y1); diff(f1, z1)]
grad_f2 = [diff(f2, x2); diff(f2, y2); diff(f2, z2)]

 

