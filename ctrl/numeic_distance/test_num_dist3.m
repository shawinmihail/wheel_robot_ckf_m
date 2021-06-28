clc
clear
close all
% rng(200)

%% set
xt = randn(1,4) + 1*[0 2 4 6];
yt = rand(1,4)  + 1*[0 2 4 6];
zt = randn(1,4);
set = [xt; yt; zt];

%% spline
M = 1/6 * [1 -3 3 -1; 4 0 -6 3; 1 3 3 -3; 0 0 0 1];
ri = [set(:, 1) set(:, 2) set(:, 3) set(:, 4)];
A = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
spline = ri * M * A;
spline

%% plot line
figure
hold on
grid on
axis equal
plot3(set(1, :), set(2, :), set(3, :),'b*')
for a = 0:0.01:1
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'g.')
end
ret

% traj
k = 0;
lst = set(:,1);
for i = 1:splines_length
spline = splines(:,:,i);
for a = 0:0.01:1
    k = k+1;  
    spline;
    sp_point = spline * [1; a; a^2; a^3];
    
    tr_point(:,k) =  lst + randn(3,1)/20 + (sp_point-lst)/10;
    lst = tr_point(:,k);
end
end
plot3(tr_point(1,:), tr_point(2,:), tr_point(3,:), 'k')


% % grid
% step = 1;
% k = 0;
% for x = 5:step:15
%     for y = -10:step:2
%         
%         k = k + 1;
%         plot(x,y,'k.')
%         
%         sstar = -1;
%         for i = 1:length(splines)
%         spline_coefs = splines(:, :, i);
%         slim = 1;
%         
%         [sstar, pstar, DELTA] = distance2spline3d([x;y;0], slim, spline_coefs);
%         if sstar > 0           
%             plot([x pstar(1)], [y pstar(2)],'g')
%             break
%         end
%         end
%         if sstar < 0
%              plot(x,y,'ro')
%         end
%     end
% end

