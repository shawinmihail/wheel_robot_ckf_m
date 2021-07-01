function [virtual_set, virtual_way, virtual_zone_center, virtual_dir] = virtual_zone_params(maneur_displacement, maneur_radius, waypoints)

set = waypoints;

vp_dir = set(:,1) - set(:,2);
vp_dir = vp_dir / norm(vp_dir);
virtual_dir = - vp_dir;
p1 = set(:,1) + vp_dir * maneur_displacement;
p2 = p1 + vp_dir * maneur_radius;
p3 = p2 + vp_dir * maneur_radius;
virtual_set = [p3 p2 p1 set(:,1)];
virtual_way = M_spline_from_set(virtual_set);
virtual_zone_center = p2;

end

