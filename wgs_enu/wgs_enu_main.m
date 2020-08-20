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

% ENU
R_ecef_enu = [-sin(lambda)            cos(lambda)             0;
     -sin(phi) * cos(lambda) -sin(phi) * sin(lambda) cos(phi);
     cos(phi) * cos(lambda)  cos(phi) * sin(lambda)  sin(phi)];
R_enu_ecef = R_ecef_enu';

% g unit
cg_unit = r_ecef / norm(r_ecef);
surf_n = (R_enu_ecef * [0;0;1]);
surf_n = surf_n / norm(surf_n);
g_angle = acos(dot(cg_unit, surf_n)) * 180 / pi;

% paper enu increment
% dphi = 0.5 * pi / 180;
% dlambda = 0.5 * pi / 180;
dh = 0;

k = 0;
lim = 0.05 * pi / 180;
step = 0.01 * pi / 180;
for dphi = -lim:step:lim
for dlambda = -lim:step:lim
for dh = -1000:100:1000
        
    k = k + 1;
        
    de = (a / xi + h) * cos(phi) * dlambda - (a * (1 - e^2) / xi^3 + h) * sin(phi) * dphi * dlambda + cos(phi) * dlambda * dh;
    dn = (a * (1 - e^2) / xi^3 + h) * dphi + 3 / 2 * a * cos(phi) * sin(phi) * e^2 * dphi^2 + dh * dphi + 1 / 2 * sin(phi) * cos(phi) * (a / xi + h) * dlambda^2;
    du = dh - 1 / 2 * a * ( 1 - 3 / 2 * e^2 * cos(phi) + 1 / 2 * e^2 + h / a) * dphi^2 - 1 / 2 * (a * cos(phi)^2 / xi - h * cos(phi)^2) * dlambda^2;
    dr_enu = [de; dn; du];

    phi1 = phi + dphi;
    lambda1 = lambda + dlambda;
    h1 = h + dh;

    xi1 = sqrt(1 - e^2 * sin(phi1)^2);
    x = (a / xi1 + h1) * cos(phi1) * cos(lambda1);
    y = (a / xi1 + h1) * cos(phi1) * sin(lambda1);
    z = (a * (1 - e^2) / xi1 + h1) * sin(phi1);
    r_ecef1 = [x;y;z];
    dr_enu1 = R_ecef_enu * (r_ecef1 - r_ecef);

    d = dr_enu1 - dr_enu;
    nd = norm(d);
    h_dist1 = [dr_enu1(1); dr_enu1(2); 0];
    xx(k) = norm(h_dist1);
    yy(k) = nd;
    
end
end
end

figure
hold on
grid on
plot(xx, yy, 'ko')

ret
% plot
[X, Y, Z] = ellipsoid(0,0,0,a,a,b,30);
figure
hold on
grid on

surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeAlpha', 0.2)
plot3(x,y,z, 'ro')
wgs_enu_plot_frame(r_ecef, R_enu_ecef, a/5);

axis equal