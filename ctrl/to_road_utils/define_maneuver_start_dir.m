function [start_u, start_v] = define_maneuver_start_dir(q, r, maneur_center, virtual_dir)

    start_u = 1;
    start_v = 1;

    ex = quatRotate(q, [1;0;0]);
    dot1 = ex(1)*virtual_dir(1) + ex(2)*virtual_dir(2);
    det1 = ex(1)*virtual_dir(2) - virtual_dir(1)*ex(2);
    angle = atan2(det1, dot1);
    
    if angle < 0
        start_u = -1;
    end
    
    dx = maneur_center - r;
    dot2 = ex(1)*maneur_center(1) + ex(2)*maneur_center(2);
    det2 = ex(1)*maneur_center(2) - maneur_center(1)*ex(2);
    angle = atan2(det1, dot1);
    
    if angle < 0
        start_v = -1;
    end
    
end

