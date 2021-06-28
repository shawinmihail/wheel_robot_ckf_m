clc
clear
close all
format long

%% read file
path = 'cam/cam.bag';
bag = rosbag(path);
bag.AvailableTopics
ts = bag.StartTime;
tf = bag.EndTime;
%% 
% obstacleDataArray
selection = select(bag,'Time',[ts tf],'Topic','/camera/obstacles');
msgs = readMessages(selection,'DataFormat','struct');
% msgs{1}
% ret
TimeTag = cellfun(@(m) double(m.TimeTag), msgs);
Checksum = cellfun(@(m) double(m.Checksum), msgs);
Valid = cellfun(@(m) double(m.Valid), msgs);
ObjectSize = cellfun(@(m) double(m.ObjectSize), msgs);

figure
hold on
grid on
axis equal
c1 = 1;
c2 = 1;
colors = {'ko', 'ro', 'go', 'bo', 'mo'};
% for i = 1:numel(msgs)
    i = 10;
    Obs = msgs{i}.Obstacles
    for k = 1:numel(Obs) 
        Ob = Obs(k)
        ret
        otype = Ob.ObjectType;
        otypes(c2) = Ob.ObjectType;
        c2 = c2 + 1;
        for m = 1:numel(Ob.BoxIndex) 
            r = cam_mes_to_r(double(Ob.BoxIndex(m)), double(Ob.BoxIndexDistance(m)));
            rlen(c1) = Ob.BoxIndexDistance(m);
            c1 = c1 + 1;
            color = colors{otype+1};
            plot3(r(1), r(2), r(3), color)
        end
    end
% end

% % PointCloud2
% selection = select(bag,'Time',[ts tf],'Topic','/pmvl');
% msgs = readMessages(selection,'DataFormat','struct');
% msgs{1};
% msgs{1}.Fields(1)
% % TimeTag = cellfun(@(m) double(m.TimeTag), msgs);
% % Checksum = cellfun(@(m) double(m.Checksum), msgs);
% % Valid = cellfun(@(m) double(m.Valid), msgs);
% % ObjectSize = cellfun(@(m) double(m.ObjectSize), msgs);