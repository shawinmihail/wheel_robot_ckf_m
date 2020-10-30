first_crop = 150;
% s1
ts_est = timeseries(estimated_array_triplet(:,7:9),timeline);
ts_mes = timeseries(dr1_mes_enu_list(1:3, :)',enu_timeline);
ts_est =  resample(ts_est, enu_timeline);
dx = ts_est.Data - ts_mes.Data;
s1_cost = vecnorm(dx');
s1_cost = s1_cost(first_crop:end);

% s2
ts_est = timeseries(estimated_array_triplet(:,10:12),timeline);
ts_mes = timeseries(dr2_mes_enu_list(1:3, :)',enu_timeline);
ts_est =  resample(ts_est, enu_timeline);
dx = ts_est.Data - ts_mes.Data;
s2_cost = vecnorm(dx');
s2_cost = s2_cost(first_crop:end);

% w
ts_est = timeseries(estimated_array_imu(:, 4:6),timeline);
ts_mes = timeseries(array_imu(:, 4:6), ts_imu);
ts_mes =  resample(ts_mes, timeline);
dx = ts_est.Data - ts_mes.Data;
w_cost = vecnorm(dx');
w_cost = w_cost(first_crop:end);

% plot(w_cost)
% QUALITY = 1 * mean(w_cost) +  max(abs(w_cost));
QUALITY = 1 * mean(s1_cost) + 1 * mean(s2_cost) + 2 * max(abs(s1_cost)) + 2 * max(abs(s2_cost));
% QUALITY = QUALITY / 6



