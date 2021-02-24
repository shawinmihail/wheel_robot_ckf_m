function oc_grid_matrix = refresh_grid(p1s, p2s, oc_grid_matrix, oc_grid_size, oc_grid_resolution)

s = size(p2s);
n = s(2);
% free
for k = 1:n
    p1 = p1s(:,k);
    p2 = p2s(:,k);
    % begin ray opened
    indexes = beginray_cells(p1, oc_grid_size, oc_grid_resolution);
    s = size(indexes);
    for i = 1:s(2)
        index = indexes(:,i);
        oc_grid_matrix(index(1), index(2)) = 1;
    end

    % on ray opened
    [indexes, xmin_d, xmax_d, ymin_d, ymax_d, x_inters_points, y_inters_points, xrays, yrays]  = onray_free_cells(p1, p2, oc_grid_size, oc_grid_resolution);
    s = size(indexes);
    for i = 1:s(2)
        index = indexes(:,i);
        oc_grid_matrix(index(1), index(2)) = 1;
    end    
end

% close
for k = 1:n
    p1 = p1s(:,k);
    p2 = p2s(:,k);

    % end ray closed
    indexes = endray_cells(p2, oc_grid_size, oc_grid_resolution);
    s = size(indexes);
    for i = 1:s(2)
        index = indexes(:,i);
        oc_grid_matrix(index(1), index(2)) = 2;
    end
end

end

