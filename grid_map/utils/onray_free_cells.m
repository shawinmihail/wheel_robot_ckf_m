function [indexes, xmin_d, xmax_d, ymin_d, ymax_d, x_inters_points, y_inters_points, xrays, yrays] = onray_free_cells(p1, p2, oc_grid_size, oc_grid_resolution)
eps = 1e-3;
indexes = [];
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution); 

dp = p2 - p1;
ndp = norm(dp);
if ndp < eps
    return
end
dir = (dp) / ndp;
tmin = 0;
tmax = norm(dp);

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

s = size(x_inters_points);
for i = 1:s(2)
    x_opened_minus = x_inters_points(:,i) + [eps; 0];
    x_opened_plus = x_inters_points(:,i) - [eps; 0];
    index = floor([x_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    if check_index(index, oc_grid_dim)
        indexes = [indexes index];
    end
    index = floor([x_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    if check_index(index, oc_grid_dim)
        indexes = [indexes index];
    end
end
s = size(y_inters_points);
for i = 1:s(2)
    y_opened_minus = y_inters_points(:,i) + [0; eps];
    y_opened_plus = y_inters_points(:,i) - [0; eps];
    index = floor([y_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    if check_index(index, oc_grid_dim)
        indexes = [indexes index];
    end
    index = floor([y_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
    if check_index(index, oc_grid_dim)
        indexes = [indexes index];
    end
end


end

