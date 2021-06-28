function res = sstar_criteria_dot_numeric_3d(spline_cfs ,s, y)
    eps = 1e-6;
    x1 = s - eps;
    x2 = s + eps;
    y1 = sstar_criteria_3d(spline_cfs ,x1, y);
    y2 = sstar_criteria_3d(spline_cfs ,x2, y);
    res = (y2-y1)/(x2-x1);
end