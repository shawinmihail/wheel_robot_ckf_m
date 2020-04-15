function [x, y, z] = wgs_cartesian_to_local(r0, ecef_x, ecef_y, ecef_z)
p0 = [r0'];
lat_lon_h = ecef2lla(p0);
lat0 = lat_lon_h(1);
lon0 = lat_lon_h(2);
h0 = lat_lon_h(3);
[x, y, z] = ecef2enu(ecef_x, ecef_y, ecef_z, lat0, lon0, h0, wgs84Ellipsoid);
