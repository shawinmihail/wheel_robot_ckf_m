function oc_grid_matrix = refresh_grid_additional(p1s, p2s, oc_grid_matrix, oc_grid_size, oc_grid_resolution)

lim = 2.0;

s = size(p2s);
n = s(2);
% on ray opened
for k = 1:n
    p1 = p1s(:,k);
    p2 = p2s(:,k);

    [indexes, xmin_d, xmax_d, ymin_d, ymax_d, x_inters_points, y_inters_points, xrays, yrays]  = onray_free_cells_with_range_lim(p1, p2, lim, oc_grid_size, oc_grid_resolution);
    s = size(indexes);
    for i = 1:s(2)
        index = indexes(:,i);
        oc_grid_matrix(index(1), index(2)) = oc_grid_matrix(index(1), index(2)) - 2;
        oc_grid_matrix(index(1), index(2)) = max(-50, oc_grid_matrix(index(1), index(2)));
    end    
end

% end ray closed
for k = 1:n
    p1 = p1s(:,k);
    p2 = p2s(:,k);
    indexes = endray_cells_with_range_lim(p1, p2, lim, oc_grid_size, oc_grid_resolution);
    s = size(indexes);
    for i = 1:s(2)
        index = indexes(:,i);
        oc_grid_matrix(index(1), index(2)) = oc_grid_matrix(index(1), index(2)) + 25;
        oc_grid_matrix(index(1), index(2)) = min(50, oc_grid_matrix(index(1), index(2)));
    end
end


end

