clc
clear
close all

fcn = @(x) x^2-10;
fcn_dot = @(x) 2*x;

x0 = 1;
for i = 1:100
    x0 = x0 - fcn(x0) / fcn_dot(x0);
    xs(i) = x0;
end

plot(xs)
