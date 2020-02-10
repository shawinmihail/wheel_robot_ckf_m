function [cubX] = get_cubature_points(sqrtP, X, N)

cubX = zeros(N, 2 * N);    
for i = 1:N
    cubX(:,i) = X + sqrt(N) * sqrtP(:, i);
end

for i = (N+1):(2*N)
    cubX(:,i) = X - sqrt(N) * sqrtP(:, i - N);
end

end