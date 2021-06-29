function [u, v, sstar, pstar, DELTA, index, error_cntr, delta_h, dot_p] = calculate_ctrl_3d_grad_pot_avoidance(y, v, C, splines, index, error_cntr, obstacle_list)
u = 0;
sstar = -1;
pstar = [0;0;0];
DELTA = [0;0;0];

delta_h = 0;
dot_p = 0;

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
rmin = 3.0;
pot_list = [[0;0]];
r = y;
q = matrix2quat(C);
dp;
% v3 = C * [1; 0; 0];
% v2 = [v3(1) v3(2)];
v2 = [dp(1) dp(2)];
v2 = v2 / norm(v2);
for i = 1:size(obstacle_list, 1)
    dr3 = obstacle_list(i,:)' - r;
    dr = [dr3(1); dr3(2); 0];
    n_dr = max(norm(dr), 0.1);
    
    dir_dr = [dr(1); dr(2)]/norm(dr);
    dot1 = v2(1)*dr(1) + v2(2)*dr(2);
    det1 = v2(1)*dr(2) - dr(1)*v2(2);
    angle = atan2(det1, dot1);
    
    if (n_dr < rmin)
        if dot1 > 0
            pot = 0.5 * dir_dr / n_dr / n_dr/ abs(angle);
            pot_list = [pot_list pot];  
        else
            pot_list = [pot_list [0;0]];
        end
    end       
end

eps = 1e-3;
du = 0;
sum_pot = sum(pot_list, 2);
if abs(sum_pot) > eps
    n_sum_pot = norm(sum_pot);

    % check zero case
    if abs(sum_pot(2)) > eps
        nv_sum_pot = [1; -sum_pot(1) / sum_pot(2)];
    else
        nv_sum_pot = [-sum_pot(2) / sum_pot(1); 1];
    end
    nv_sum_pot = nv_sum_pot / norm(nv_sum_pot);

    % check dir?
    dot_p = dot(v2, nv_sum_pot);

    % check sign
    rot_dir = sign(dot_p) * nv_sum_pot;
    rod_dir_B = C' * [rot_dir; 0];
    du_sign = sign(rod_dir_B(2));

    du = du_sign * n_sum_pot;
end
u = u + du;



lim = 20*pi/180;
lim = 1;
u = min(lim, max(-lim, u));
error_cntr = 0;



DELTA_B = C'*DELTA;
delta_h = sign(DELTA_B(2)) * norm(DELTA_B .* [1;1;0]);

ev = C * [1; 0; 0];
ev(3) = 0;
v2 = [dp(1); dp(2); 0];
v2 = v2 / norm(v2);
dot_p = dot(ev, v2);

