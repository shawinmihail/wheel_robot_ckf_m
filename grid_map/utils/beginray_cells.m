function indexes = beginray_cells(p1, oc_grid_size, oc_grid_resolution)

eps = oc_grid_resolution/10;
indexes = [];
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
grid_lim = oc_grid_size/2 - eps;
if p1(1) > grid_lim || p1(1) < -grid_lim
    return
end
if p1(2) > grid_lim || p1(2) < -grid_lim
    return
end

p1_h = [p1(1); p1(2)];
[i1, i2, i3, i4] = x_area_indexes_of_point(p1_h, eps, oc_grid_size, oc_grid_resolution);

% x area
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

