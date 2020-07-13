function [astar, sstar, d] = distance2spline(y, spline, a)

d_spline = diff(spline, a);
eq = (y - spline)'*d_spline == 0;
astars = vpasolve(eq, a);

astars_real = astars(imag(astars)==0);
astars = astars_real(astars_real > 0 & astars_real < 1);
if isempty(astars)
   astar = -1;
   sstar = [0; 0];
   d = -1;
   return
end

astar = astars(1);
sstar = subs(spline, astars);
d = norm(y - sstar);

end