clc
close all
load('D:\!MATLAB\wheel_robot_ckf_m\logs\mes_q.mat')

fs = 18;
%% q
% figure
% hold on
% title('Ориентация')
% plot(timeline, est_states(10, :), 'k')
% plot(timeline, est_states(11, :), 'r')
% plot(timeline, est_states(12, :), 'g')
% plot(timeline, est_states(13, :), 'b')

% plot(timeline, mes_q(1, :), 'k--')
% plot(timeline, mes_q(2, :), 'r--')
% plot(timeline, -mes_q(3, :), 'g--')
% plot(timeline, -mes_q(4, :), 'b--')
% set(gca,'FontSize',fs)

for i = 1:length(est_states(11, :))
    e(:, i) = quat2Eul(est_states(10:13, i)) * 180 / pi;
end

for i = 1:length(mes_q(1, :))
    e0(:, i) = quat2Eul(mes_q(1:4, i)) * 180 / pi;
end


figure
hold on
de = e(1, :) - e0(1, :);
plot(timeline(1500:end), de(1500:end), 'r--')
de = e(2, :) - e0(2, :);
plot(timeline(1500:end), de(1500:end), 'g--')
de = e(3, :) - e0(3, :);
plot(timeline(1500:end), de(1500:end), 'b--')
ylim([-10 10])