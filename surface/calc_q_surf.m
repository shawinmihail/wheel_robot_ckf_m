function q_surf = calc_q_surf(n_unit)
eps = 1e-8;
%% q_surf
ez = [0;0;1];
cos_alpha = dot(ez, n_unit);
alpha = acos(cos_alpha);
if abs(alpha) < eps
    q_surf = [1;0;0;0];
    return
end
cos_half_alpha = cos(alpha/2);
sin_half_alpha = sin(alpha/2);
pin = cross(ez, n_unit);
pin = pin / norm(pin);
q_surf = [cos_half_alpha; pin * sin_half_alpha];
end

