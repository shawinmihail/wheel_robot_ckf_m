function [smoothed, data] = ekf4_smooth(sample, data, N)

data = [sample data];
len = length(data(1,:));
if len > N
    data = data(:, 1:end-1);
    len = len - 1;
end

s = 0;
smoothed = sample*0;
for n = 1:len
    K = 1 + len - n;
    K = 1;
    s = s + K;
    smoothed = smoothed + K*data(:,n);
end
smoothed = smoothed / s;

end

