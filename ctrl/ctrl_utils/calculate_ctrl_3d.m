function [u, sstar, pstar, DELTA] = calculate_ctrl_3d(y, v, C, splines, obstacle_list)
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
%     phi = acos(cosPhi)
%     sinPhi = sin(phi);
%     sinPhi = sqrt(1-cosPhi^2);
%     sinPhi = dp' * C * [1; 0; 0] / norm_ps;
    nvect = cross(C * [1; 0; 0], DELTA);
    crossSign = sign(nvect(3));
%     u = -crossSign*(delta*lambda^2 + 2*lambda*cosPhi + (Delta_pss*sinPhi^2)/(delta*(-norm_ps^2 + Delta_pss)))/sinPhi;
    
%%
    rmin = 2.5;
    Pi_mult = 1.0;

    poten = 0;
    potens = 0;
    potenss = 0;
    potenss_u = 0;
    r = y;
    for i = 1:size(obstacle_list, 1)
        dr = obstacle_list(i,:)' - r;
        ndr = max(norm(dr), 0.05);
        ndr_s(i) = ndr;
        cos_theta = dr' * C * [1; 0; 0] / ndr;
        sin_sign = sign(cross(dr, C * [1; 0; 0]));
        sin_sign = dot(sin_sign', [0,0,1]);
        %cos_theta = dr' * [1; 0; 0] / ndr;
        ndrs = - v*cos_theta;
        sin_theta = sqrt(1 - cos_theta^2)*sin_sign;
        if (ndr < rmin) & (cos_theta > 0)
    %         poten = poten + 0.5 * (rmin - ndr)^2;
    %         potens = potens +(rmin - ndr)*v*cos_theta;
    %         %potenss = potenss + ndrs^2 - (rmin - ndr)*v*v*sin_theta;
    %         potenss = potenss + ndrs^2;
    %         potenss_u = - (rmin - ndr)*v*v*sin_theta;
            poten = poten + 1/ndr;
            potens = potens + v*cos_theta/ndr^2;
            potenss = potenss + 2*v^2*cos_theta^2/ndr^3;
            potenss_u = potenss_u - v^2*sin_theta/ndr^2;
        end       

    end

    Pi = -poten;
    Pis = -potens;
    mu = lambda;
    signX = crossSign;
    phi = acos(cosPhi);
    u = surf_ctrl_pot_test_1(Delta_pss,Pi,Pi_mult,Pis,delta,mu,norm_ps,phi,signX);
    
%%
    lim = 20*pi/180;
%     lim = 1;
    u = min(lim, max(-lim, u));
else
    'warn check dist'
%     ret
end