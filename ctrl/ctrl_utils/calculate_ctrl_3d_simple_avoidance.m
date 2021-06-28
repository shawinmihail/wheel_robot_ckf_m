function [u, v, sstar, pstar, DELTA, index, error_cntr] = calculate_ctrl_3d_simple_avoidance(y, v, C, splines, index, error_cntr, obstacle_list)
u = 0;
sstar = -1;
pstar = [0;0;0];
DELTA = [0;0;0];

if index < 1
msg = 'spline index error, index < 1'
return
end
if index > length(splines)
msg = 'spline index error, index > len'
return
end

if error_cntr > 50 && index == length(splines)
msg = 'end of route, stop'
v = 0;
return
end
if error_cntr > 50 && index == 1
msg = 'not in the beging of the road, stop'
v = 0;
return
end
if error_cntr > 50 && index ~= length(splines)
msg = 'check distance error, stop'
v = 0;
return
end

spline_coefs = splines(:, :, index);
slim = 1;
[sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_coefs);

if sstar < 0
    if index ~= length(splines)
        spline_coefs = splines(:, :, index+1);
        slim = 1;
        [sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_coefs);
        if sstar < 0
            error_cntr = error_cntr + 1;
            return
        else
            index = index + 1;
        end
    else
        error_cntr = error_cntr + 1;
        return;
    end
end

% DELTA_z_error_param = 1.5;
% if abs(DELTA(3)) < 1.5
%     DELTA(3) = DELTA(3) * (1 - (DELTA_z_error_param - norm([DELTA(1) DELTA(2)]))/DELTA_z_error_param);
% end

DELTA_z_error_param = 1.5;
if abs(DELTA(3)) < 1.5
    DELTA(3) = 0;
end

lambda = 1;
delta = norm(DELTA);
D = C' * DELTA;
p = spline_point_3d(spline_coefs, sstar);
dp = spline_dot_point_3d(spline_coefs, sstar);
ddp = spline_ddot_point_3d(spline_coefs, sstar);
sstar_dot = v * [1 0 0] * C' * dp / (norm(dp)^2 - DELTA' * ddp);

norm_ps = norm(dp);
Delta_pss = DELTA' * ddp;
cosPhi = DELTA' * C * [1; 0; 0] / delta;
sinPhi = sqrt(1-cosPhi^2);
nvect = cross(C * [1; 0; 0], DELTA);
crossSign = sign(nvect(3));
u = -crossSign*(delta*lambda^2 + 2*lambda*cosPhi + (Delta_pss*sinPhi^2)/(delta*(-norm_ps^2 + Delta_pss)))/sinPhi;

%%
rmin = 4.0;
du_mult = 1.0;
du = [];
r = y;
for i = 1:size(obstacle_list, 1)
    dr3 = obstacle_list(i,:)' - r;
    dr = [dr3(1); dr3(2); 0];
    ndr = max(norm(dr), 0.1);
    ndr_s(i) = ndr;
    
%     v3 = C * [1; 0; 0];
%     v2 = [v3(1) v3(2)];
    
    v2 = [dp(1) dp(2)];
    v2 = v2 / norm(v2);
    
    p_obs = dr;
    dot1 = v2(1)*p_obs(1) + v2(2)*p_obs(2);
    det1 = v2(1)*p_obs(2) - p_obs(1)*v2(2);
    angle = atan2(det1, dot1);
    
    if (ndr < rmin)
        dui  = du_mult / angle  / ndr^2;
%         dui  = du_mult / angle /10;
        du = [du -dui];
    end       
end

val = 0;
val_with_max_abs = 0;
if length(du) > 0
[~, max_ind] = max(abs(du));
val_with_max_abs = du(max_ind);
top_indexes = find(abs(du) > 1*abs(val_with_max_abs)/3);
top_values = du(top_indexes);
[~, min_ind] = min(abs(top_values));
val = top_values(min_ind);
end
u = u + sum(du);
% u = u + min(du);
% u = u + val_with_max_abs;

lim = 20*pi/180;
u = min(lim, max(-lim, u));
error_cntr = 0;
