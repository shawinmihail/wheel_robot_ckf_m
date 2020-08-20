clc
close all
clear
format long

% params
a =  6378137;
f = 1/298.257223563;
b = a*(1 - f);
e = sqrt(1 - b^2 / a^2);

% initial
phi = 0 * pi / 180; % lat
lambda = 0 * pi / 180; % lon
h = 0; % alt

% ECEF
xi = sqrt(1 - e^2 * sin(phi)^2);
x = (a / xi + h) * cos(phi) * cos(lambda);
y = (a / xi + h) * cos(phi) * sin(lambda);
z = (a * (1 - e^2) / xi + h) * sin(phi);
r_ecef = [x;y;z];

% R
R_ecef_enu = [-sin(lambda)            cos(lambda)             0;
     -sin(phi) * cos(lambda) -sin(phi) * sin(lambda) cos(phi);
     cos(phi) * cos(lambda)  cos(phi) * sin(lambda)  sin(phi)];
R_enu_ecef = R_ecef_enu';

% g unit
cg_unit = r_ecef / norm(r_ecef);
surf_n = (R_enu_ecef * [0;0;1]);
surf_n = surf_n / norm(surf_n);
g_angle = acos(dot(cg_unit, surf_n)) * 180 / pi;

% ENU
dphi = 0.1 * pi / 180;
dlambda = 0.0 * pi / 180;
dh = 0;

phi1 = phi + dphi;
lambda1 = lambda + dlambda;
h1 = h + dh;

k = 0;
lim = 0.0008 * pi / 180;
step = 0.0001 * pi / 180;
for dphi = -lim:step:lim
for dlambda = -lim:step:lim
for dh = 0:1:0
    
    k = k + 1;
    
    phi1 = phi + dphi;
    lambda1 = lambda + dlambda;
    h1 = h + dh;
    
    xi1 = sqrt(1 - e^2 * sin(phi1)^2);
    x1 = (a / xi1 + h1) * cos(phi1) * cos(lambda1);
    y1 = (a / xi1 + h1) * cos(phi1) * sin(lambda1);
    z1 = (a * (1 - e^2) / xi1 + h1) * sin(phi1);
    r_ecef1 = [x1;y1;z1];
    
    dr_ecef = r_ecef1 - r_ecef;
    dr_enu = R_ecef_enu * dr_ecef;
    
    
    xx(k) = norm([dr_enu(1); dr_enu(2); 0]);
    yy(k) = norm(dr_enu(3));

end
end
end

figure
hold on
grid on
plot(xx, yy, 'ko')


