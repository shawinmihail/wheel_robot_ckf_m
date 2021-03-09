clc
clear
close all
% rng(505)


% oc grid initial
oc_grid_size = 32;
oc_grid_resolution = 2;
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution); 
oc_grid_size = oc_grid_dim * oc_grid_resolution;
oc_grid_matrix = zeros(oc_grid_dim, oc_grid_dim);
oc_grid_matrix(1,1) = 1;
oc_grid_matrix(1,2) = 2;
lim = 1;

% segment
p2 = [1 + 5 *randn();1 + 5 *randn();0];
p1 = [1 + 5 *randn();1 + 5 *randn();0];
% p1 = [-15.9;0;0];
% p2 = [17;1;0];
dp = p2 - p1;

% begin ray opened
indexes = beginray_cells(p1, oc_grid_size, oc_grid_resolution);
s = size(indexes);
for i = 1:s(2)
    index = indexes(:,i);
    oc_grid_matrix(index(1), index(2)) = 1;
end

% on ray opened
[indexes, xmin_d, xmax_d, ymin_d, ymax_d, x_inters_points, y_inters_points, xrays, yrays]  = onray_free_cells(p1, p2, oc_grid_size, oc_grid_resolution);
s = size(indexes);
for i = 1:s(2)
    index = indexes(:,i);
    oc_grid_matrix(index(1), index(2)) = 1;
end


% end ray closed
indexes = endray_cells(p2, oc_grid_size, oc_grid_resolution);
s = size(indexes);
for i = 1:s(2)
    index = indexes(:,i);
    oc_grid_matrix(index(1), index(2)) = 2;
end


%% plot
figure
grid on
hold on
% xlim([-3 3])
% ylim([-3 3])
axis equal

% matrix
% h = imagesc(oc_grid_matrix');
[X,Y] = meshgrid([-oc_grid_size/2:oc_grid_resolution:oc_grid_size/2]);
surf(X,Y,X*0, oc_grid_matrix')

% ray
plot(p1(1), p1(2), 'r*')
plot(p2(1), p2(2), 'r*')
plot([p1(1) ,p2(1)], [p1(2) ,p2(2)], 'r')

% borders
plot([xmin_d ,xmax_d], [ymin_d ,ymin_d], 'g', 'LineWidth', 3)
plot([xmin_d ,xmax_d], [ymax_d ,ymax_d], 'g', 'LineWidth', 3)
plot([xmin_d ,xmin_d], [ymin_d ,ymax_d], 'g', 'LineWidth', 3)
plot([xmax_d ,xmax_d], [ymin_d ,ymax_d], 'g', 'LineWidth', 3)

% checklines
for i = 1:length(xrays)
    x = xrays(i);
    plot([x ,x], [ymin_d ,ymax_d], 'm--', 'LineWidth', 2)
end
for i = 1:length(yrays)
    y =yrays(i);
    plot([xmin_d ,xmax_d], [y ,y], 'm--', 'LineWidth', 2)
end

% inters points
s = size(x_inters_points);
for i = 1:s(2)
     plot(x_inters_points(1,i), x_inters_points(2,i), 'r*', 'LineWidth', 2)
end
s = size(y_inters_points);
for i = 1:s(2)
     plot(y_inters_points(1,i), y_inters_points(2,i), 'r*', 'LineWidth', 2)
end


% plot(linemin(1), linemin(2), 'r*')
% plot(linemax(1), linemax(2), 'r*')
