function [sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_cfs)

sstar = callc_sstar_numeric2(y, spline_cfs, slim);
if sstar < 0
   sstar = -1;
   pstar = [0; 0; 0];
   DELTA = [0;0;0];
   return
end

pstar = spline_point_3d(spline_cfs, sstar);
DELTA = y - pstar;
end