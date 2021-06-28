function [u, v, sstar, pstar, DELTA, index, error_cntr] = calculate_ctrl_3d_in_area(y, v, C, splines, index, error_cntr, obstacle_list)
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

if error_cntr > 20 & index ~= 1
msg = 'check distance error, stop'
v = 0;
return
end

if error_cntr > 20 && index == 1
msg = 'not in the beging of the road, stop'
v = 0;
ret
return
end

if index == length(splines)
msg = 'end of the road, stop'
v = 0;
return;
end

% area
area_indexes = [index];
total_length = 0;
max_length = 1.0;
area_index_0 = index + 1;
while total_length < max_length
    
    if area_index_0 > length(splines)
        break;
    end
    
    area_indexes = [area_indexes area_index_0];
    spline_coefs = splines(:, :, area_index_0);
    p1 = spline_point_3d(spline_coefs, 0);
    p2 = spline_point_3d(spline_coefs, 1);
    d = norm(p2-p1);
    total_length = total_length + d;
    area_index_0 = area_index_0 + 1;
end

% try to find sstar
sstar = -1;
pstar = [0;0;0];
DELTA = [0;0;0];
index_found = -1;
spline_coefs = zeros(3,4);
for i = 1:length(area_indexes)
    index_found = area_indexes(i);
    spline_coefs = splines(:, :, index_found);
    slim = 1;
    [sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_coefs);
    if sstar > 0
        break;
    end
end

% check we found
if sstar < 0
    error_cntr = error_cntr + 1;
    return;
end

index = index_found;
% ctrl
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
