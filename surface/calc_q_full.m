function q = calc_q_full(grad_surf, r, yaw)
n_unit = grad_surf(r);
n_unit = n_unit / norm(n_unit);
q_surf = calc_q_surf(n_unit);
q_yaw = calc_q_yaw(yaw, n_unit);
q = quatMultiply(q_yaw, q_surf);
end

