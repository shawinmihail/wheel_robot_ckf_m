function point = spline_point_3d(spline_cfs, s)
point = [spline_cfs(1,1) * s^3 + spline_cfs(1,2) * s^2 + spline_cfs(1,3) * s + spline_cfs(1,4);
         spline_cfs(2,1) * s^3 + spline_cfs(2,2) * s^2 + spline_cfs(2,3) * s + spline_cfs(2,4);
         spline_cfs(3,1) * s^3 + spline_cfs(3,2) * s^2 + spline_cfs(3,3) * s + spline_cfs(3,4)];
end