clc
clear
close all
% rng(201)
eps = 1e-3;


% oc grid initial
oc_grid_size = 32;
oc_grid_resolution = 1;
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution); 
oc_grid_size = oc_grid_dim * oc_grid_resolution;
oc_grid_matrix = zeros(oc_grid_dim, oc_grid_dim);
% oc_grid_matrix = randn(oc_grid_dim, oc_grid_dim);

% segment
p2 = [1 + 5 *randn();1 + 5 *randn();0];
p1 = [1 + 5 *randn();1 + 5 *randn();0];
p1 = [-1;1.5;0];
p2 = [14;-9;0];
dp = p2 - p1;

dir = (dp) / norm(dp);
t = 0;
tmin = 0;
tmax = norm(dp);
line = p1 + dir * t;
linemin = p1 + dir * tmin;
linemax = p1 + dir * tmax;

% find borders
xmin = min(p1(1), p2(1));
xmax = max(p1(1), p2(1));
ymin = min(p1(2), p2(2));
ymax = max(p1(2), p2(2));

xmin = max(-oc_grid_size/2 + eps, xmin);
xmax = min(oc_grid_size/2 - eps, xmax);
ymin = max(-oc_grid_size/2 + eps, ymin);
ymax = min(oc_grid_size/2 - eps, ymax);

xmin_d = floor((xmin + oc_grid_size/2) / oc_grid_resolution) * oc_grid_resolution - oc_grid_size/2;
xmax_d = ceil((xmax + oc_grid_size/2) / oc_grid_resolution) * oc_grid_resolution - oc_grid_size/2;
ymin_d = floor((ymin + oc_grid_size/2) / oc_grid_resolution) * oc_grid_resolution - oc_grid_size/2;
ymax_d = ceil((ymax + oc_grid_size/2) / oc_grid_resolution) * oc_grid_resolution - oc_grid_size/2;


% find intrs
xrays = xmin_d+oc_grid_resolution:oc_grid_resolution:xmax_d-oc_grid_resolution;
yrays = ymin_d+oc_grid_resolution:oc_grid_resolution:ymax_d-oc_grid_resolution;

x_inters_points = [];
if abs(dir(1)) > eps
for i = 1:length(xrays)
    x = xrays(i);
    t = (x - p1(1)) / dir(1);
    if t > tmin & t < tmax
        y = p1(2) + dir(2)*t;
        x_inters_points = [x_inters_points [x;y]];
    end
end
end
y_inters_points = [];
if abs(dir(2)) > eps
for i = 1:length(yrays)
    y = yrays(i);
    t = (y - p1(2)) / dir(2);
    if t > tmin & t < tmax
        x = p1(1) + dir(1)*t;
        y_inters_points = [y_inters_points [x;y]];
    end
end
end

% find opened
p1_h = [p1(1); p1(2)];
x_opened_minus = p1_h + [-eps; eps];
x_opened_plus = p1_h + [eps; eps];
y_opened_minus = p1_h + [-eps; -eps];
y_opened_plus = p1_h + [eps; -eps];
index = floor([x_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
% TODO check index
oc_grid_matrix(index(1), index(2)) = -1;
index = floor([x_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = -1;
index = floor([y_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = -1;
index = floor([y_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = -1;


s = size(x_inters_points);
for i = 1:s(2)
    x_opened_minus = x_inters_points(:,i) + [eps; 0];
    x_opened_plus = x_inters_points(:,i) - [eps; 0];
    index = floor([x_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    oc_grid_matrix(index(1), index(2)) = -1;
    index = floor([x_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    oc_grid_matrix(index(1), index(2)) = -1;
end
s = size(y_inters_points);
for i = 1:s(2)
    y_opened_minus = y_inters_points(:,i) + [0; eps];
    y_opened_plus = y_inters_points(:,i) - [0; eps];
    index = floor([y_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    oc_grid_matrix(index(1), index(2)) = -1;
    index = floor([y_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    oc_grid_matrix(index(1), index(2)) = -1;
end

% find closed
p2_h = [p2(1); p2(2)];
x_opened_minus = p2_h + [-eps; eps];
x_opened_plus = p2_h + [eps; eps];
y_opened_minus = p2_h + [-eps; -eps];
y_opened_plus = p2_h + [eps; -eps];
index = floor([x_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = 1;
index = floor([x_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = 1;
index = floor([y_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = 1;
index = floor([y_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
oc_grid_matrix(index(1), index(2)) = 1;


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
