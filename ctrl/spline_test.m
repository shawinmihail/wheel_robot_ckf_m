clc
clear
close all

t = 0:1:20;
traj = [t; 2 * sin( 2 * t / 2 / pi); t/1000];
traj_dubler = [t+0.5; 1.9 * sin( 2.1 * (t+0.5) / 2 / pi) + 0.3; t/700];

figure
hold on
grid on
% xlim([0 3])
% ylim([0 3])
plot(traj(1, :), traj(2, :), 'b*')
plot(traj_dubler(1, :), traj_dubler(2, :), 'g*')


M = 1/6 * [1 -3 3 -1; 4 0 -6 3; 1 3 3 -3; 0 0 0 1];
for i = 2:length(t)-3
ri = [traj(:, i-1) traj(:, i) traj(:, i+1) traj(:, i+2)];
A = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
spline = ri * M * A;
% splines(:, :, i) = spline;
for a = 0:0.1:1
    spline_point = spline * [1; a; a^2; a^3];
    plot3(spline_point(1), spline_point(2), spline_point(3), 'ko')
end

end

% for k = 1:length(traj_dubler)
%     y = traj_dubler(:,k);
%     for i = 2:length(t)-3
%         spline = splines(:, i);
%         [astar, sstar, d] = distance2spline(y, spline, a);
%         astar
%         if astar < 1 & astar > 0
%             plot([y(1) sstar(1)], [y(2) sstar(2)])
%             
%             v = 1;
%             lambda = 2;
%         end
%     end
% end
