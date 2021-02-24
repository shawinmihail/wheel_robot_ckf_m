function res = check_index(index, oc_grid_dim)
if index(1) > oc_grid_dim || index(2) > oc_grid_dim
    res = 0;
    return
end
res = 1;
return
end

