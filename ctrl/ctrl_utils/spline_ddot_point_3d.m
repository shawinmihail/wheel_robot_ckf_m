function point = spline_ddot_point_3d(spline_cfs, s)
point = [6 * spline_cfs(1,1) * s + 2 * spline_cfs(1,2);
         6 * spline_cfs(2,1) * s + 2 * spline_cfs(2,2);
         6 * spline_cfs(3,1) * s + 2 * spline_cfs(3,2)];
end