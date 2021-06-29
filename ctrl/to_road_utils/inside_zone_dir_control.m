function [u, v, mode, mode_fixed, angle] = inside_zone_dir_control(q, r, virtual_dir, virtual_way, maneur_center, maneur_radius, dir_angle_eps, maneur_radius_eps, steer_lim, mode, mode_fixed, start_u, start_v)

ex = quatRotate(q, [1;0;0]);
dot1 = ex(1)*virtual_dir(1) + ex(2)*virtual_dir(2);
det1 = ex(1)*virtual_dir(2) - virtual_dir(1)*ex(2);
angle = atan2(det1, dot1);

u = start_v*start_u*steer_lim*mode;
v = start_v*mode;

r_h = [r(1); r(2); 0];
maneur_center_h = [maneur_center(1); maneur_center(2); 0];
if mode_fixed == 0
if norm(r_h-maneur_center_h) > maneur_radius + maneur_radius_eps
    mode = mode * -1;
    mode_fixed = 1;
end
else
    if norm(r_h-maneur_center_h) < maneur_radius - maneur_radius_eps
        mode_fixed = 0;
    end
end