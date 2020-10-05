clc
clear
close all

t = 0:1:3;
traj = [t; 2 * sin( 2 * t / 2 / pi); 0.01*t];
traj_dubler = [t+0.5; 1.5 * sin( 2.5 * (t+0.5) / 2 / pi) + 0.3; 0.01*t];

figure
hold on
grid on
% xlim([0 3])
% ylim([0 3])
% plot3(traj(1, :), traj(2, :), traj(3, :), 'b*')
% plot3(traj_dubler(1, :), traj_dubler(2, :), traj_dubler(3, :), 'g*')

traj_spline_obj = cscvn(traj);
traj_dubler_spline_obj = cscvn(traj_dubler);


syms s
assume(s, 'real')
splines = zeros(3, length(t))*s;
cfs = traj_spline_obj.coefs;
bks = traj_spline_obj.breaks;

for i = 1:length(t)-1

n = (i-1)*3 + 1;
spline = [cfs(n, 1)*s^3 + cfs(n, 2)*s^2 + cfs(n, 3)*s + cfs(n, 4);
          cfs(n+1, 1)*s^3 + cfs(n+1, 2)*s^2 + cfs(n+1, 3)*s + cfs(n+1, 4);
          cfs(n+2, 1)*s^3 + cfs(n+2, 2)*s^2 + cfs(n+2, 3)*s + cfs(n+2, 4)];

splines(:, i) = spline;

spline_cfs = [cfs(n, 1) cfs(n, 2) cfs(n, 3) cfs(n, 4);
                 cfs(n+1, 1) cfs(n+1, 2) cfs(n+1, 3) cfs(n+1, 4);
                 cfs(n+2, 1) cfs(n+2, 2) cfs(n+2, 3) cfs(n+2, 4)];

% for S = 0:0.1:bks(i+1)-bks(i)
%     point = spline_point_3d(spline_cfs, S);
%     plot3([point(1)], [point(2)], [point(3)], 'r.')
% end

end

for k = 1:length(traj_dubler)
    y = traj_dubler(:,k);
    for i = 1:length(t)-1
        
        n = (i-1)*3 + 1;
        spline_cfs = [cfs(n, 1) cfs(n, 2) cfs(n, 3) cfs(n, 4);
                     cfs(n+1, 1) cfs(n+1, 2) cfs(n+1, 3) cfs(n+1, 4);
                     cfs(n+2, 1) cfs(n+2, 2) cfs(n+2, 3) cfs(n+2, 4)];
        spline = splines(:, i);
        
        slim = bks(i+1)-bks(i);
        [sstar, pstar, DELTA] = distance2spline3d(y, slim, spline_cfs);
    
        if sstar > 0
            
            plot3([y(1) pstar(1)], [y(2) pstar(2)], [y(3) pstar(3)])
            
            v = 1;
            lambda = -2;
            delta = norm(DELTA);
            C = [1 0 0; 0 1 0; 0 0 1];
            D = C' * DELTA;
            
            d_spline = diff(spline);
            dd_spline = diff(d_spline);
            dp = vpa(subs(d_spline, sstar));
            ddp = vpa(subs(dd_spline, sstar));
            sstar_dot = v * [1 0 0] * C' * dp / (norm(dp)^2 - DELTA' * ddp);
            
            u = (-lambda^2 * delta^2 - 2*v*lambda*D(1) - v^2 + v * sstar_dot * dp' * C * [1;0;0] + (v*D(1)/ delta)^2) ...
            / (v^2 * D(2));
        end
    end
end
