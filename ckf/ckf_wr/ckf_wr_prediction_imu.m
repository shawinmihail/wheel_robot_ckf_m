function [X_predicted, sqrtP_predicted]  = ckf_wr_prediction_imu(X, sqrtP, sqrtQ, a_mes, w_mes, dt)

% X = [r v q]
N = numel(X);

%% state prediction
X_cub_points = get_cubature_points(sqrtP, X, N);
X_cub_points_evaluted = zeros(N, 2 * N);
for i = 1:(2*N)
    X_cub_points_evaluted(:,i) = ckf_wr_evalution_imu(X_cub_points(:,i), a_mes, w_mes, dt);
end
X_predicted = 1 / (2*N) * sum(X_cub_points_evaluted, 2);
% X_predicted = evalution(X, a_mes, w_mes, dt);

%% state prediction covariance
hi_predicted = 1 / (sqrt(2*N)) * (X_cub_points_evaluted - X_predicted(:,ones(1,(2*N))));
sqrtP_predicted = tria([hi_predicted, sqrtQ], N);
