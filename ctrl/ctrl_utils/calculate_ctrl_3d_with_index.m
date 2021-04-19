function [u, v, sstar, pstar, DELTA, index, error_cntr] = calculate_ctrl_3d_with_index(y, v, C, splines, index, error_cntr)
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
% if error_cntr > 50 && index ~= length(splines)
% msg = 'check distance error, stop'
% v = 0;
% return
% end

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

lambda = 1;
delta = norm(DELTA);
D = C' * DELTA;
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

lim = 1;
u = min(lim, max(-lim, u));
error_cntr = 0;
