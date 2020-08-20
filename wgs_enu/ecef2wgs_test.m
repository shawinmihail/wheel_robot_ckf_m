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
h = 3; % alt

% ECEF
xi = sqrt(1 - e^2 * sin(phi)^2);
x = (a / xi + h) * cos(phi) * cos(lambda);
y = (a / xi + h) * cos(phi) * sin(lambda);
z = (a * (1 - e^2) / xi + h) * sin(phi);
r_ecef = [x;y;z];



k = 0;
limdphi = 90 * pi / 180;
limdlambda = 180 * pi / 180;
step = 1 * pi / 180;
for dphi = -limdphi:step:limdphi
for dlambda = 0:step:limdlambda
for dh = -1000:100:1000
        
    k = k + 1;
        
    phi1 = phi + dphi;
    lambda1 = lambda + dlambda;
    h1 = h + dh;

    xi1 = sqrt(1 - e^2 * sin(phi1)^2);
    x = (a / xi1 + h1) * cos(phi1) * cos(lambda1);
    y = (a / xi1 + h1) * cos(phi1) * sin(lambda1);
    z = (a * (1 - e^2) / xi1 + h1) * sin(phi1);
    [phi2, lambda2, h2] = ecef2wgs(x, y, z);
    
    xx(k) = k;
    yy1(k) = phi2 - phi1;
    yy2(k) = lambda2 - lambda1;
    yy3(k) = h2 - h1;
    
end
end
end

figure
hold on
grid on
plot(xx, yy1, 'ko')

figure
hold on
grid on
plot(xx, yy2, 'ko')

figure
hold on
grid on
plot(xx, yy3, 'ko')