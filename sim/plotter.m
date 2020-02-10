%% get data
x_est = est_states(1, :);
y_est = est_states(2, :);
z_est = est_states(3, :);
vx_est = est_states(4, :);
vy_est = est_states(5, :);
vz_est = est_states(6, :);
qw_est = est_states(7, :);
qx_est = est_states(8, :);
qy_est = est_states(9, :);
qz_est = est_states(10, :);

x_mes = mes_states(1, :);
y_mes = mes_states(2, :);
z_mes = mes_states(3, :);
vx_mes = mes_states(4, :);
vy_mes = mes_states(5, :);
vz_mes = mes_states(6, :);
ax_mes = mes_states(7, :);
ay_mes = mes_states(8, :);
az_mes = mes_states(9, :);
qw_mes = mes_states(10, :);
qx_mes = mes_states(11, :);
qy_mes = mes_states(12, :);
qz_mes = mes_states(13, :);
wx_mes = mes_states(14, :);
wy_mes = mes_states(15, :);
wz_mes = mes_states(16, :);

x_act = act_states(1, :);
y_act = act_states(2, :);
z_act = act_states(3, :);
vx_act = act_states(4, :);
vy_act = act_states(5, :);
vz_act = act_states(6, :);
ax_act = act_states(7, :);
ay_act = act_states(8, :);
az_act = act_states(9, :);
qw_act = act_states(10, :);
qx_act = act_states(11, :);
qy_act = act_states(12, :);
qz_act = act_states(13, :);
wx_act = act_states(14, :);
wy_act = act_states(15, :);
wz_act = act_states(16, :);

%% PLOTTER
% figure
% hold on
% plot(sqrtP_diag(3, :), 'r')


figure
hold on
% plot(qw_est, 'k--')
% plot(qw_act, 'k')
plot(qx_est, 'r')
plot(qx_act, 'r--')
plot(qy_est, 'g')
plot(qy_act, 'g--')
plot(qz_est, 'b')
plot(qz_act, 'b--')

% figure
% hold on
% plot(x_est, y_est, 'k')
% plot(x_act, y_act, 'k--')
% plot(x_est, 'k')
% plot(x_act, 'k--')

% figure
% hold on
% % plot(ax_mes, 'r')
% % plot(ax_act, 'k')
% % plot(ax_est, y_est, 'r')
% plot(qx_est, 'r')
% plot(qx_act, 'k')