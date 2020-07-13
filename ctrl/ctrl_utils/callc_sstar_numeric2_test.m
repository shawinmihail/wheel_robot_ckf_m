clc
clear 
close all

y = [1 0 1]';
slim = 1.22;
spline_cfs = [-0.0031         0    1.2252         0;
              0.0257   -0.0000    0.0550   -1.0000;
              0.0006    0.0000    0.0013    0.9988];
          
sstar1 = callc_sstar_numeric(y, spline_cfs, slim)

step = 1e-1;
eps = 1e-3;

figure
hold on
x0 = 1;
plot(i,x0, 'go')
fx0 = 0;
fx0_dot = 0;
for i = 1:5
    fx0 = sstar_criteria_3d(spline_cfs ,x0, y);
    fx0_dot = sstar_criteria_dot_3d(spline_cfs ,x0, y);
    x0 = x0 - fx0/fx0_dot;
    plot(i,fx0, 'ro')
    if x0 < 0 || x0 > slim
        break
    end
end
if abs(fx0) < eps && x0 > 0 && x0 < slim
    sstar = x0
    return
end

 sstar = -1;
