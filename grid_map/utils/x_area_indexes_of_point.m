function [i1, i2, i3, i4] = x_area_indexes_of_point(p, eps, oc_grid_size, oc_grid_resolution)
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
p1 = p + [-eps; eps];
p2 = p + [eps; eps];
p3 = p + [-eps; -eps];
p4 = p + [eps; -eps];
i1 = index_of_point(p1, oc_grid_size, oc_grid_resolution);
i2 = index_of_point(p2, oc_grid_size, oc_grid_resolution);
i3 = index_of_point(p3, oc_grid_size, oc_grid_resolution);
i4 = index_of_point(p4, oc_grid_size, oc_grid_resolution);
end

