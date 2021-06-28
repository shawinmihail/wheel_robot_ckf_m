function res = check_index(index, oc_grid_dim)
if index(1) > oc_grid_dim || index(2) > oc_grid_dim || index(1) < 1 || index(2) < 1
    res = 0;
    return
end
res = 1;
end

