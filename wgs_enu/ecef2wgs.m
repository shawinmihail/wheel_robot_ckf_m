function [phi, lambda, h] = ecef2wgs(x, y, z)

a =  6378137;
f = 1/298.257223563;
b = a*(1 - f);
e = sqrt(1 - b^2 / a^2);

w_2 = x^2 + y^2 ;
l = e^2 / 2 ;
m = w_2 / a^2 ;
n = z^2 * (1-e^2) / a^2 ;
p = (m + n - 4*l^2) / 6 ;
G = m*n*l^2 ;
m = w_2 / a^2 ;
H = 2*p^3 + G;
% if
C = (H + G + 2 * sqrt(H*G))^(1/3) / 2^(1/3) ;
i = -(2*l^2 + m + n) / 2 ;
P = p^2 ;
beta = i/3 - C - P/C ;
k = l^2 * (l^2 - m - n) ;
t = sqrt(sqrt(beta^2-k) - (beta + i)/2) - sign(m-n) * sqrt(abs(beta - i) / 2) ;
F = t^4 + 2*i*t^2 + 2*l*(m-n)*t + k ;
dFdt = 4*t^3 + 4*i*t + 2*l * (m-n) ;
Delta_t = -F / (dFdt) ;
u = t + Delta_t + l;
v = t + Delta_t - l;
w = sqrt(w_2) ;
phi = atan2(z*u, w*v) ;
Delta_w = w*(1-1/u) ;
Delta_z = z*(1-(1 - e^2)/v) ;
h = sign(u -1) * sqrt ((Delta_w)^2 + (Delta_z)^2) ;
lambda = atan2 (y, x);


end

