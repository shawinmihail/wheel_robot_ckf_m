function [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, splines)
u = 0;
sstar = -1;
pstar = [0;0;0];
DELTA = [0;0;0];
for i = 1:length(splines)
    
        spline_coefs = splines(:, :, i);
        slim = 1;
        
        [sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_coefs);
        if sstar > 0           
            break
        end
end

if sstar > 0

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
%     sinPhi = dp' * C * [1; 0; 0] / norm_ps;
    crossSign = sign(cross(DELTA, C * [1; 0; 0]));
    u = -crossSign*(delta*lambda^2 + 2*lambda*cosPhi + (Delta_pss*sinPhi^2)/(delta*(-norm_ps^2 + Delta_pss)))/sinPhi;
    
%     eps = 1e-1;
%     if norm(D) < eps
%         lambda = norm(D)*10;
%     end
    
%     u = (-lambda^2 * delta^2 - 2*v*lambda*D(1) - v^2 + v * sstar_dot * dp' * C * [1;0;0] + (v*D(1)/ delta)^2) ...
%     / (v^2 * D(2));
% 
    lim = 1;
    u = min(lim, max(-lim, u));
end