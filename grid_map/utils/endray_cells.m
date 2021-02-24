function indexes = endray_cells(p2, oc_grid_size, oc_grid_resolution)

eps = 1e-3;
indexes = [];
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
grid_lim = oc_grid_size/2 - eps;
if p2(1) > grid_lim || p2(1) < -grid_lim
    return
end
if p2(2) > grid_lim || p2(2) < -grid_lim
    return
end

% find closed
p2_h = [p2(1); p2(2)];
x_opened_minus = p2_h + [-eps; eps];
x_opened_plus = p2_h + [eps; eps];
y_opened_minus = p2_h + [-eps; -eps];
y_opened_plus = p2_h + [eps; -eps];
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

