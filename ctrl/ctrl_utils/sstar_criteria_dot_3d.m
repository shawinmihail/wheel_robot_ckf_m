function res = sstar_criteria_dot_3d(spline_cfs ,s, y)
    res = -spline_dot_point_3d(spline_cfs ,s)' * spline_dot_point_3d(spline_cfs ,s) + ...
        (y - spline_point_3d(spline_cfs ,s))' * spline_ddot_point_3d(spline_cfs ,s);
end