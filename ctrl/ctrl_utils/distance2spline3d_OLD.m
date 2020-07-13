function [sstar, pstar, DELTA] = distance2spline3d(y, spline, s, slim, spline_cfs)

d_spline = diff(spline, s);
eq = (y - spline)'*d_spline == 0;
sstars = vpasolve(eq, s);

sstars_real = sstars(imag(sstars)==0);
sstars = sstars_real(sstars_real > 0 & sstars_real < slim);

if isempty(sstars)
   sstar = -1;
   pstar = [0; 0; 0];
   DELTA = [0;0;0];
   return
end


sstar = sstars(1)
eps = 1e-2;
sstar_num = callc_sstar_numeric(y, spline_cfs, slim, eps)


pstar = vpa(subs(spline, sstar));
DELTA = y - pstar;
end