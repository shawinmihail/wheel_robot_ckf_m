function [status] = define_status(status, q, r, maneur_center, maneur_radius, maneur_radius_eps, virtual_dir, angle_eps, in_domain)

% 1 initial
% 2 inside zone
% 3 directed inside zone
% 4 on way
% -1 error

if status == 1
    % check if we inside radius
    r_h = [r(1); r(2); 0];
    maneur_center_h = [maneur_center(1); maneur_center(2); 0];
    if norm(r_h - maneur_center_h) > maneur_radius - maneur_radius_eps
        status = -1;
        msg = 'not in initial radius, end';
    else
        status = 2;
    end
end

if status == 2
    ex = quatRotate(q, [1;0;0]);
    dot1 = ex(1)*virtual_dir(1) + ex(2)*virtual_dir(2);
    det1 = ex(1)*virtual_dir(2) - virtual_dir(1)*ex(2);
    angle = atan2(det1, dot1);
    if abs(angle) < angle_eps
        status = 3;
    end
end

if status == 3
    if in_domain == 1
        status = 4;
    end
end


end

