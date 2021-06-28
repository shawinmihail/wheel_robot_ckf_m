function [i1, i2] = h_area_indexes_of_point(p, eps, oc_grid_size, oc_grid_resolution)
p1 = p + [-eps; 0];
p2 = p + [eps; 0];
i1 = index_of_point(p1, oc_grid_size, oc_grid_resolution);
i2 = index_of_point(p2, oc_grid_size, oc_grid_resolution);
end

