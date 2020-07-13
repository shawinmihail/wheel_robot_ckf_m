clc
clear
close all

lambda = 1;
v = 1;
C = eye(3);
dp = [1; 0; 0];
ddp = [0.01; 0.01; 0];

figure
hold on
for x = -1e-7:1e-8:1e-7
    for y = -0.1:0.01:0.1
    DELTA = [x; y; 0];
    delta = norm(DELTA);
    sstar_dot = v * [1 0 0] * C' * dp / (norm(dp)^2 - DELTA' * ddp)
    u = (-lambda^2 * delta^2 - 2*v*lambda*x - v^2 + v * sstar_dot * dp' * C * [1;0;0] + (v*x/ delta)^2) ...
    / (v^2 * y)

    plot3(x,y,u, 'ko')
    end
end