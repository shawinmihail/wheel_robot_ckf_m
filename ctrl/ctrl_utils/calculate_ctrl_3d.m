function [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, cfs, bks)
u = 0;
sstar = -1;
pstar = [0;0;0];
DELTA = [0;0;0];
for i = 1:length(cfs(:, 1)) / 3
    
        n = (i-1)*3 + 1;
        spline_coefs = [cfs(n, 1) cfs(n, 2) cfs(n, 3) cfs(n, 4);
                        cfs(n+1, 1) cfs(n+1, 2) cfs(n+1, 3) cfs(n+1, 4);
                        cfs(n+2, 1) cfs(n+2, 2) cfs(n+2, 3) cfs(n+2, 4)];
    
        slim = bks(i+1)-bks(i);
        
        [sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_coefs);
        if sstar > 0           
            break
        end
end

if sstar > 0

    
    lambda = 2;
    delta = norm(DELTA);
    D = C' * DELTA;
    dp = spline_dot_point_3d(spline_coefs, sstar);
    ddp = spline_ddot_point_3d(spline_coefs, sstar);
    sstar_dot = v * [1 0 0] * C' * dp / (norm(dp)^2 - DELTA' * ddp);
    
%     eps = 1e-1;
%     if norm(D) < eps
%         lambda = norm(D)*10;
%     end
    
    u = (-lambda^2 * delta^2 - 2*v*lambda*D(1) - v^2 + v * sstar_dot * dp' * C * [1;0;0] + (v*D(1)/ delta)^2) ...
    / (v^2 * D(2));

    lim = 1;
    u = min(lim, max(-lim, u));
end