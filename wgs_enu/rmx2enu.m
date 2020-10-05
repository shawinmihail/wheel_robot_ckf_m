function R_ecef_enu = rmx2enu(phi, lambda)
R_ecef_enu = [-sin(lambda)            cos(lambda)             0;
              -sin(phi) * cos(lambda) -sin(phi) * sin(lambda) cos(phi);
              cos(phi) * cos(lambda)  cos(phi) * sin(lambda)  sin(phi)];
end
