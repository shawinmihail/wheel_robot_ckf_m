function res = sstar_criteria_3d(spline_cfs ,s, y)
    res = (y - spline_point_3d(spline_cfs ,s))' * spline_dot_point_3d(spline_cfs ,s);
end