function indexes = endray_cells(p2, oc_grid_size, oc_grid_resolution)

eps = oc_grid_resolution/10;
indexes = [];
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
grid_lim = oc_grid_size/2 - eps;
if p2(1) > grid_lim || p2(1) < -grid_lim
    return
end
if p2(2) > grid_lim || p2(2) < -grid_lim
    return
end

p2_h = [p2(1); p2(2)];
[i1, i2, i3, i4] = x_area_indexes_of_point(p2_h, eps, oc_grid_size, oc_grid_resolution);

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

