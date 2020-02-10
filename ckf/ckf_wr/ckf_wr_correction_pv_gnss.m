function [X, sqrtP]  = ckf_wr_correction_pv_gnss(X, sqrtP, sqrtR, Z)

% X = [r v q]
% Z = [r v]

N = numel(X);
M = numel(Z);

% Evaluate cubature points
X_cub_points = get_cubature_points(sqrtP, X, N);

% Evaluate propagated cubature points
Z_cub_points = zeros(M, 2*N);
for i = 1:(2*N)
    Z_cub_points(:,i) = ckf_wr_state2mes_pv_gnns(X_cub_points(:,i));
end

% Estimate the predicted measurement
Z_predicted = 1 / (2*N) * sum(Z_cub_points, 2);

% Estimate the square-root of the innovation covariance matrix
Z_matrix = 1 / (sqrt(2*N)) * (Z_cub_points - Z_predicted(:,ones(1,(2*N))));
sqrtS = tria([Z_matrix, sqrtR], M);

% Estimate the cross-covariance matrix 
hi_new = 1 / (sqrt(2*N)) * (X_cub_points - X(:,ones(1,(2*N))));
Pxz = hi_new * Z_matrix';

% Estimate Kalman gain 
K = (Pxz / (sqrtS')) / sqrtS;

% Estimate the updated state
dz = Z - Z_predicted;
X = X + K * dz;
sqrtP = tria([hi_new - K * Z_matrix, K * sqrtR], N);


