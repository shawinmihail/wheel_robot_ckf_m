function indexes = beginray_cells(p1, oc_grid_size, oc_grid_resolution)

eps = 1e-3;
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
x_opened_minus = p1_h + [-eps; eps];
x_opened_plus = p1_h + [eps; eps];
y_opened_minus = p1_h + [-eps; -eps];
y_opened_plus = p1_h + [eps; -eps];
index = floor([x_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
if check_index(index, oc_grid_dim)
    indexes = [indexes index];
end
index = floor([x_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; x_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
if check_index(index, oc_grid_dim)
    indexes = [indexes index];
end
index = floor([y_opened_minus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_minus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
if check_index(index, oc_grid_dim)
    indexes = [indexes index];
end
index = floor([y_opened_plus(1) / oc_grid_resolution + oc_grid_dim / 2; y_opened_plus(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
if check_index(index, oc_grid_dim)
    indexes = [indexes index];
end
end

