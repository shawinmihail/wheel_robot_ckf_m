clc
clear
close all

% syms z2
% 
% u = 1;
% f1 = u*sqrt(1 - z2^2);
% f2 = -u*sqrt(1 - z2^2);
% 
% lambda = 1;
% z1 = 0.1;
% k = 1;
% g = -k / (1 - k * z1) * z2^2 - 2 * lambda * z2 - lambda^2 * z1 + k / (1 - k * z1);
% 
% hold on
% fplot(f1,[-3 1])
% fplot(f2,[-3 1])
% fplot(g,[-3 1])
% ret

alpha = 0.66;
lambda = 2;

k = 0;
u_lim = 0.5;

i = 0;
for z1 = 0:0.02:alpha/abs(k) + 0
    for z2 = -1:0.02:1
        
        i = i + 1;
        
        u = 1 / sqrt( 1 - z2^2) * (- lambda^2 * z1 - 2 * lambda * z2 + k * (1 - z2^2) / (1 - k * z1) );
        u1 = 1 / sqrt( 1 - z2^2) * (- lambda^2 * z1 - 2 * lambda * z2);
        u2 =  (k * sqrt(1 - z2^2) / (1 - k * z1) );
        
        f = u_lim * sqrt(1-z2^2);
        
        k = abs(k);
        g_plus = (- lambda^2 * z1 - 2 * lambda * z2 + k * (1 - z2^2) / (1 - k * z1) );
        g_plus_plane = (- lambda^2 * z1 - 2 * lambda * z2);
        g_plus_ksurf = (k * (1 - z2^2) / (1 - k * z1) );
        g_plus_ksurf_esimate = k / (1 - alpha);

        
        k = -abs(k);
        g_minus = (- lambda^2 * z1 - 2 * lambda * z2 + k * (1 - z2^2) / (1 - k * z1) );
        g_minus_plane = (- lambda^2 * z1 - 2 * lambda * z2);
        g_minus_ksurf = (k * (1 - z2^2) / (1 - k * z1) );
        g_minus_ksurf_esimate = k;
        
        x(i) = z1;
        y(i) = z2;
        
        h1(i) = f;
        h2(i) = -f;
        h3(i) = g_plus;
        h4(i) = g_minus;
        
        plane_plus = g_plus_plane + g_plus_ksurf_esimate;
        h5(i) = plane_plus;
        
        plane_minus = g_minus_plane + g_minus_ksurf_esimate;
        h6(i) = plane_minus;
        h7(i) = g_minus;
    end
end

figure
hold on
grid on
% xlim([-3 3])
% ylim([-3 3])
% zlim([-3 3])
view([90 90])

xlabel('z1')
ylabel('z2')
zlabel('y')

orange = [0.99, 0.6, 0.5];
% plot_3d_surf(x,y,h1, 'r')
plot_3d_surf(x,y,h2, 'b')
% plot_3d_surf(x,y,h3, 'm')
% plot_3d_surf(x,y,h4, 'g')
% plot_3d_surf(x,y,h5, orange)
plot_3d_surf(x,y,h6, 'g')
% plot_3d_surf(x,y,h7, 'k')

figure
hold on
grid on
% xlim([-3 3])
% ylim([-3 3])
% zlim([-3 3])
view([90 90])

xlabel('z1')
ylabel('z2')
zlabel('y')

orange = [0.99, 0.6, 0.5];
plot_3d_surf(x,y,h1, 'r')
% plot_3d_surf(x,y,h2, 'b')
% plot_3d_surf(x,y,h3, 'm')
% plot_3d_surf(x,y,h4, 'g')
plot_3d_surf(x,y,h5, orange)
% plot_3d_surf(x,y,h6, 'g')

