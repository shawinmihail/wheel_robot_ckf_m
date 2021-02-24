clc
clear
close all
format long

%% read file
path1 = '_2021-02-18-21-18-33.bag';
bag1 = rosbag(path1);
% bag1.AvailableTopics
ts1 = bag1.StartTime;
tf1 = bag1.EndTime;
%% 
% mes
selection1 = select(bag1,'Time',[ts1 tf1],'Topic','/scan');
msgs1 = readMessages(selection1,'DataFormat','struct');
stamp0 = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec) / 1e9, msgs1);
stamp = stamp0 - stamp0(1);
AngleMin = cellfun(@(m) double(m.AngleMin), msgs1);
AngleMax = cellfun(@(m) double(m.AngleMax), msgs1);
AngleIncrement = cellfun(@(m) double(m.AngleIncrement), msgs1);
TimeIncrement = cellfun(@(m) double(m.TimeIncrement), msgs1);
ScanTime = cellfun(@(m) double(m.ScanTime), msgs1);
RangeMin = cellfun(@(m) double(m.RangeMin), msgs1);
RangeMax = cellfun(@(m) double(m.RangeMax), msgs1);
Ranges = cellfun(@(m) double(m.Ranges), msgs1, 'UniformOutput', false); 
Intensities = cellfun(@(m) double(m.Intensities), msgs1, 'UniformOutput', false);  
%%
speed = 8;


for i = 1:size(Ranges,1)
x = (AngleMin(i):AngleIncrement(i):AngleMax(i)+AngleIncrement(i))'+pi;
y = Ranges{i,:};
% polarplot(x,y); 

% end
%%
% rej_r2 = 1.9;
% rej_r1 = 1.1;
% y((y>rej_r2)|(y<rej_r1)) = Inf;

obs_theta = x(~isinf(y));
obs_r = y(~isinf(y));

% rej = 2*pi/3;
% rej_ = pi/60;
% obs_r = obs_r((obs_theta<(pi-rej))|(obs_theta>(pi+rej+rej_)));
% obs_theta = obs_theta((obs_theta<(pi-rej))|(obs_theta>(pi+rej+rej_)));

h = polarplot(obs_theta, obs_r,'.', 'LineWidth', 2);
rlim([0 4])

if (i == 1)
    pause(stamp(i)/speed);
else
    pause((stamp(i)-stamp(i-1))/speed);
end
end

