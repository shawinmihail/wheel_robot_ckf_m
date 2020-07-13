clc
clear
close all

t = 0:1:20;
traj = [t; 2 * sin( 2 * t / 2 / pi)];
traj_dubler = [t+0.5; 1.9 * sin( 2.1 * (t+0.5) / 2 / pi) + 0.3];

figure
hold on
grid on
% xlim([0 3])
% ylim([0 3])
plot(traj(1, :), traj(2, :), 'b*')
plot(traj_dubler(1, :), traj_dubler(2, :), 'g*')

syms a
assume(a, 'real')
M = 1/6 * [1 -3 3 -1; 4 0 -6 3; 1 3 3 -3; 0 0 0 1];
splines = zeros(2, length(t))*a;
for i = 2:length(t)-3
ri = [traj(:, i-1) traj(:, i) traj(:, i+1) traj(:, i+2)];
A = [1; a; a^2; a^3];
spline = vpa(ri * M * A);

splines(:, i) = spline;
end

for k = 1:length(traj_dubler)
    y = traj_dubler(:,k);
    for i = 2:length(t)-3
        spline = splines(:, i);
        [astar, sstar, d] = distance2spline(y, spline, a);
        astar
        if astar < 1 & astar > 0
            plot([y(1) sstar(1)], [y(2) sstar(2)])
            
            v = 1;
            lambda = 2;
        end
    end
end
