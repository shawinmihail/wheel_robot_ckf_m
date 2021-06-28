clc
clear
close all

% % spline
xt = [
8 10 8 6 4 20 2
];
yt = [
-10 -8 -6 -4 -2 0 2
];
zt = 0 * xt;
set = [xt; yt; zt];
[splines] = M_spline_from_set(set);

% plot spline
figure
hold on
grid on
axis equal
plot3(set(1, :), set(2, :), set(3, :),'b*')
splines_length = length(splines);
for i = 1:splines_length

colors = {'r.' 'g.' 'b.' 'k.' 'c.' 'm.'};
spline = splines(:,:,i);
for a = 0:0.01:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), colors{i})
end
end

% % traj
% k = 0;
% lst = set(:,1);
% for i = 1:splines_length
% spline = splines(:,:,i);
% for a = 0:0.01:1
%     k = k+1;  
%     spline;
%     sp_point = spline * [1; a; a^2; a^3];
%     
%     tr_point(:,k) =  lst + randn(3,1)/20 + (sp_point-lst)/10;
%     lst = tr_point(:,k);
% end
% end
% plot3(tr_point(1,:), tr_point(2,:), tr_point(3,:), 'k')


% grid
step = 1;
k = 0;
for x = 5:step:15
    for y = -10:step:2
        
        k = k + 1;
        plot(x,y,'k.')
        
        sstar = -1;
        for i = 1:length(splines)
        spline_coefs = splines(:, :, i);
        slim = 1;
        
        [sstar, pstar, DELTA] = distance2spline3d([x;y;0], slim, spline_coefs);
        if sstar > 0           
            plot([x pstar(1)], [y pstar(2)],'g')
            break
        end
        end
        if sstar < 0
             plot(x,y,'ro')
        end
    end
end

