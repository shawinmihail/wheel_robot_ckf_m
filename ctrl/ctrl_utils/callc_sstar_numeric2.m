function sstar = callc_sstar_numeric2(y, spline_cfs, slim)

step = slim / 5;
eps = 1e-3;
for s = 0:step:slim
    x0 = s;
    fx0 = 0;
    fx0_dot = 0;
    for i = 1:5
        fx0 = sstar_criteria_3d(spline_cfs ,x0, y);
        fx0_dot = sstar_criteria_dot_3d(spline_cfs ,x0, y);
        x0 = x0 - fx0/fx0_dot;
%         if x0 < 0 || x0 > slim
%             break
%         end
    end
    if abs(fx0) < eps && x0 > -eps && x0 < slim + eps
        sstar = x0;
        return
    end
end
    sstar = -1;
end