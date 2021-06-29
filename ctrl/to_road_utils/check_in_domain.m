function in_domain = check_in_domain(q, r, virtual_dir, spline_cfs)

in_domain = 0;

ex = quatRotate(q, [1;0;0]);
dot1 = ex(1)*virtual_dir(1) + ex(2)*virtual_dir(2);
det1 = ex(1)*virtual_dir(2) - virtual_dir(1)*ex(2);
angle = atan2(det1, dot1);

sstar = callc_sstar_numeric(r, spline_cfs, 1);
if (sstar > 0) && (abs(angle) < pi/3)
    in_domain = 1;
end
    
end

