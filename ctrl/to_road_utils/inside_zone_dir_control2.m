function [u, v, mode, s] = inside_zone_dir_control2(dr, s, start_u, start_v, mode)

s = s + norm(dr);
if s >= pi/4
    s = 0;
    mode = -mode;
end

u = mode*start_u;
v = mode*start_v;

end

