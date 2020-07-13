function point = spline_dot_point_3d(spline_cfs, s)
point = [3 * spline_cfs(1,1) * s^2 + 2 * spline_cfs(1,2) * s + spline_cfs(1,3);
         3 * spline_cfs(2,1) * s^2 + 2 * spline_cfs(2,2) * s + spline_cfs(2,3);
         3 * spline_cfs(3,1) * s^2 + 2 * spline_cfs(3,2) * s + spline_cfs(3,3)];
end