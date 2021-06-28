function [index] = index_of_point(p, oc_grid_size, oc_grid_resolution)
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution);
index = floor([p(1) / oc_grid_resolution + oc_grid_dim / 2; p(2) / oc_grid_resolution + oc_grid_dim / 2]) + [1;1];
end

