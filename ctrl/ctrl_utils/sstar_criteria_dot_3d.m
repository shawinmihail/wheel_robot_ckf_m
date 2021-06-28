function res = sstar_criteria_dot_3d(spline_cfs ,s, y)
    x = (y - spline_point_3d(spline_cfs ,s))' * spline_dot_point_3d(spline_cfs ,s);
    xs = -spline_dot_point_3d(spline_cfs ,s)' * spline_dot_point_3d(spline_cfs ,s) + ...
        (y - spline_point_3d(spline_cfs ,s))' * spline_ddot_point_3d(spline_cfs ,s);
    res = xs;
end