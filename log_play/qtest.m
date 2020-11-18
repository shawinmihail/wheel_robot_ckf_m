ts_est = timeseries(estimated_array_triplet(:,7:9),timeline);
ts_mes = timeseries(dr1_mes_enu_list(1:3, :)',enu_timeline);

ts_est =  resample(ts_est, enu_timeline);
dx = ts_est.Data - ts_mes.Data;
n_dx = vecnorm(dx');
n_dx = n_dx(100:end);
plot(n_dx)
QUALITY = mean(n_dx);




