function res = sstar_criteria_3d(spline_cfs ,s, y)
    eps = 1e-3;
    DELTA = y - spline_point_3d(spline_cfs ,s);
    tangent = spline_dot_point_3d(spline_cfs ,s);
    n_DELTA = norm(DELTA);
    n_tangent = norm(tangent);
    if (n_DELTA < eps || n_tangent < eps)
        res = 0;
        return;
    end
%     res = DELTA' * tangent / n_DELTA / n_tangent;
    res = DELTA' * tangent;
%     res = res^2;
end