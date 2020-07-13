function sstar = callc_sstar_numeric(y, spline_cfs, slim)

step = 1e-3;
eps = 1e-3;
for s = 0:step:slim
    candidate = sstar_criteria_3d(spline_cfs ,s, y);
    if abs(candidate) < eps
        sstar = s;
        return
    end
end
    sstar = -1;
end