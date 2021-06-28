function [i1, i2, i3, i4] = v_area_indexes_of_point(p, eps, oc_grid_size, oc_grid_resolution)
p1 = p + [0; -eps];
p2 = p + [0; eps];
i1 = index_of_point(p1, oc_grid_size, oc_grid_resolution);
i2 = index_of_point(p2, oc_grid_size, oc_grid_resolution);
end

