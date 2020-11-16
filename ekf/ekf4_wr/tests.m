% figure
% hold on

phi = 0;
lambda = pi/2;

R_ecef_enu = [-sin(lambda)            cos(lambda)             0;
              -sin(phi) * cos(lambda) -sin(phi) * sin(lambda) cos(phi);
              cos(phi) * cos(lambda)  cos(phi) * sin(lambda)  sin(phi)];
v = [1;0;0]
v = R_ecef_enu * v
% r_mes_enu = R_ref * (r_mes - r_ref);