function point = spline_ddot_point_3d(spline_cfs, s)
point = [2*spline_cfs(1,3) + 6*spline_cfs(1,4) * s;
         2*spline_cfs(2,3) + 6*spline_cfs(2,4) * s;
         2*spline_cfs(3,3) + 6*spline_cfs(3,4) * s];
end