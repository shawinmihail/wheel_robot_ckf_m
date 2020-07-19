clc
close all
fs = 20;

% figure
% grid on
% hold on

% % title('Траектория xyz')
% [X,Y] = meshgrid(-100:1:100);  
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% Z = surf_fcn(X,Y);
% surf(X,Y,Z, 'FaceAlpha',0.7, 'EdgeColor', 'None')
% plot_frame(initial_r, initial_q)
% plot3(act_states(1,:),act_states(2,:),act_states(3,:), 'k--');
% set(gca,'FontSize',fs)
% view(-30,30)
% lim = 50;
% xlim([-lim + 10, lim + 10])
% ylim([-10 - lim, -10 + lim])
% zlim([-lim + 10, lim + 10])

% figure
% hold on
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% set(gca,'FontSize',fs)
% 
% [X,Y] = meshgrid(-19:0.5:19);
% Z = surf_fcn(X,Y);
% 
% splines_length = length(cfs(:, 1)) / 3;
% 
% z = [];
% for i = 1:splines_length
% 
% n = (i-1)*3 + 1;
% spline_cfs = [cfs(n, 1) cfs(n, 2) cfs(n, 3) cfs(n, 4);
%                  cfs(n+1, 1) cfs(n+1, 2) cfs(n+1, 3) cfs(n+1, 4);
%                  cfs(n+2, 1) cfs(n+2, 2) cfs(n+2, 3) cfs(n+2, 4)];
% 
% for S = 0:0.1:bks(i+1)-bks(i)
%     point = spline_point_3d(spline_cfs, S);
%     z = [z, point];
% %     plot3(point(1), point(2), point(3), 'r.')
% end
% 
% end
% 
% 
% p_robot = plot3(act_states(1,:),act_states(2,:),act_states(3,:), 'r', 'LineWidth', 2);
% p_target = plot3(z(1, :), z(2, :), z(3, :), 'k--', 'LineWidth', 2);
% surf(X,Y,Z, 'FaceAlpha',0.7, 'EdgeColor', 'None')
% legend([p_robot p_target],{'robot traj','target traj'})

%% q
figure
hold on
title('Position error')
xlabel('time, s')
ylabel('error, m')
set(gca,'FontSize',fs)
plot(timeline, 0.01 * randn(size(timeline)), 'k')
plot(timeline, est_states(1, :)-act_states(1, :), 'r')
% plot(timeline, est_states(12, :)-act_states(12, :), 'g')
% plot(timeline, est_states(13,:)-act_states(13, :), 'b')
set(gca,'FontSize',fs)
ylim([-0.05 0.05])
legend({'measured','filtered'})

