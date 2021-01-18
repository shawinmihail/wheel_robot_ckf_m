clc
clear
close all

rLat = 55;
rLon = 33;
rAlt = 130;

[x0, y0, z0] = wgs2ecef(rLat, rLon, rAlt);
[x1, y1, z1] = wgs2ecef(rLat, rLon, rAlt + 5);

R_ecef_enu = rmx2enu(rLat, rLon);

re = R_ecef_enu * ([x1;y1;z1] - [x0;y0;z0]);