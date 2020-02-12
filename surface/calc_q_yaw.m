function q_yaw = calc_q_yaw(yaw, n_unit)
%% q_yaw
sin_half_yaw = sin(yaw/2);
cos_half_yaw = cos(yaw/2);
q_yaw = [cos_half_yaw; n_unit*sin_half_yaw];

end

