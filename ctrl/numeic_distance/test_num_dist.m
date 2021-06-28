clc
clear
close all

% % spline
xt = [
8 8 8 9 10 11 14
];
yt = [
-10 -6 -2 -1 -0.5 -0.5 -0.5
];
zt = 0 * xt;
set = [xt; yt; zt];
[splines] = M_spline_from_set(set);

% plot spline
figure
hold on
grid on
axis equal
plot3(set(1, :), set(2, :), set(3, :),'b*')
splines_length = length(splines);
for i = 1:splines_length

colors = {'r.' 'g.' 'b.' 'k.' 'c.' 'm.'};
spline = splines(:,:,i);
for a = 0:0.01:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), colors{i})
end
end

% grid
step = 1;
k = 0;
for x = 7:step:8
    for y = -9:step:0
        
        k = k + 1;
        plot(x,y,'k.')
        
        sstar = -1;
        for i = 1:length(splines)
        spline_coefs = splines(:, :, i);
        slim = 1;
        
        [sstar, pstar, DELTA] = distance2spline3d([x;y;0], slim, spline_coefs);
        if sstar > 0           
%             plot([x pstar(1)], [y pstar(2)],'g')
            lambda = 0.45;
            delta = norm(DELTA);
            q = [0.707106781186548,0,0,0.707106781186547]';
            C = quat2matrix(q);
            D = C' * DELTA;
            dp = spline_dot_point_3d(spline_coefs, sstar);
            ddp = spline_ddot_point_3d(spline_coefs, sstar);
            norm_ps = norm(dp);
            Delta_pss = DELTA' * ddp;
            cosPhi = DELTA' * C * [1; 0; 0] / delta;
            sinPhi = sqrt(1-cosPhi^2);
            %     sinPhi = dp' * C * [1; 0; 0] / norm_ps;
            crossV = cross(DELTA, C * [1; 0; 0]);
            crossSign = -sign(crossV(3));
            u = -crossSign*(delta*lambda^2 + 2*lambda*cosrc/wr_surf_control_test.cppsPhi + (Delta_pss*sinPhi^2)/(delta*(-norm_ps^2 + Delta_pss)))/sinPhi
            break
        end
        end
        if sstar < 0
             plot(x,y,'ro')
        end
    end
end


figure
hold on
x0 = 13.9;
y0 = -1.7;
i = 6;
spline = splines(:, :, i);
for a = -1:0.01:2
    res = sstar_criteria_3d(spline, a, [x0;y0;0]);
    plot(a,res,'k.')
end

