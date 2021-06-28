function [indexes, xmin_d, xmax_d, ymin_d, ymax_d, x_inters_points, y_inters_points, xrays, yrays] = onray_free_cells_with_range_lim(p1, p2, lim, oc_grid_size, oc_grid_resolution)
eps = oc_grid_resolution/10;
indexes = [];
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution); 

dp = p2 - p1;
ndp = norm(dp);
if ndp < eps
    return
end
dir = (dp) / ndp;

if ndp > lim
    ndp = lim;
    p2 = p1 + dir*ndp;
    dp = p2 - p1;
end

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
    i1 = [-1; -1];
    i2 = [-1; -1];
    i3 = [-1; -1];
    i4 = [-1; -1];
%     [i1 i2 i3 i4] = x_area_indexes_of_point(x_inters_points(:,i), eps, oc_grid_size, oc_grid_resolution);
    [i1, i2] = h_area_indexes_of_point(x_inters_points(:,i), eps, oc_grid_size, oc_grid_resolution);
    if check_index(i1, oc_grid_dim)
        indexes = [indexes i1];
    end
    if check_index(i2, oc_grid_dim)
        indexes = [indexes i2];
    end
    if check_index(i3, oc_grid_dim)
        indexes = [indexes i3];
    end
    if check_index(i4, oc_grid_dim)
        indexes = [indexes i4];
    end
end
s = size(y_inters_points);
for i = 1:s(2)
    i1 = [-1; -1];
    i2 = [-1; -1];
    i3 = [-1; -1];
    i4 = [-1; -1];
%     [i1 i2 i3 i4] = x_area_indexes_of_point(y_inters_points(:,i), eps, oc_grid_size, oc_grid_resolution);
    [i1, i2] = v_area_indexes_of_point(y_inters_points(:,i), eps, oc_grid_size, oc_grid_resolution);
    if check_index(i1, oc_grid_dim)
        indexes = [indexes i1];
    end
    if check_index(i2, oc_grid_dim)
        indexes = [indexes i2];
    end
    if check_index(i3, oc_grid_dim)
        indexes = [indexes i3];
    end
    if check_index(i4, oc_grid_dim)
        indexes = [indexes i4];
    end
end

indexes;


end

