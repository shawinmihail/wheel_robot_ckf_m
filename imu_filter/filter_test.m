clc
close all
% clear

% Fs = 50;
% t = ts_imu;
% x = array_imu(:, 4);
% xdft = fft(x);
% 
% df = Fs/length(x);
% half_res = df/2;
% freq = -Fs/2+half_res:df:Fs/2-half_res;
% indxs = find(abs(freq) > 10 & abs(freq) < 20);
% xdft(indxs) = 0;
% % plot(freq, abs(xdft))
% % ret
% 
% xhat = ifft(xdft);
% 
% figure
% hold on
% grid on
% plot(t,xhat, 'k')
% plot(t,x, 'r')

    


a = array_imu(1:999, 1:3);
w = array_imu(1:999, 4:6);


N = 50;
y = a(:,1);
data = [];
y_f2 = [];

smoothed2 = 0;
K = 0.1;
for k = 1:length(a(:,1))
    [smoothed, data] = ekf4_smooth(a(k,1), data, 50);
%     y_f(k,:) = smoothed;
    
    smoothed2 = smoothed2 + K * (a(k,1) - smoothed2);
    y_f(k,:) = smoothed2;
end

figure
hold on
grid on
plot(y, 'k')
plot(y_f, 'r')