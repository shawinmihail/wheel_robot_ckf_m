function [x, y, z] = wgs2ecef(phi, lambda, h)
% params
a =  6378137;
f = 1/298.257223563;
b = a*(1 - f);
e = sqrt(1 - b^2 / a^2);

% ECEF
xi = sqrt(1 - e^2 * sin(phi)^2);
x = (a / xi + h) * cos(phi) * cos(lambda);
y = (a / xi + h) * cos(phi) * sin(lambda);
z = (a * (1 - e^2) / xi + h) * sin(phi);
end
