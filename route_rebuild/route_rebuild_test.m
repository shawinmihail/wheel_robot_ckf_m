clc
clear
close all
eps = 1e-3;
d_crv = 2;
d_obs = 1;
th = 0:pi/33:2*pi;
xs_obs = d_obs * cos(th);
ys_obs = d_obs * sin(th);
zs_obs = d_obs * 0*th;
xs_crv = d_crv * cos(th);
ys_crv = d_crv * sin(th);
zs_crv = d_crv * 0*th;

figure
hold on
axis equal

obstacles = [-20; 16; -2];
plot3(obstacles(1, :), obstacles(2, :), obstacles(3, :),'ko')
for k = 1:size(obstacles, 2)
    plot3(obstacles(1, k) + xs_obs, obstacles(2, k) + ys_obs, obstacles(3, k) + zs_obs, 'k')
end


tb = readtable('wps3.csv');
set = tb{1:25,1:end};
set = set';
[splines] = M_spline_from_set(set);

plot3(set(1, :), set(2, :), set(3, :),'b*')
splines_s = size(splines);
for i = 1:splines_s(3)

spline = splines(:,:,i);

for a = 0:0.1:1
    spline;
    point = spline * [1; a; a^2; a^3];
%     plot3(point(1), point(2), point(3), 'r.')
end
end

% alg
log = [];
for i = 1:splines_s(3)

spline = splines(:,:,i);

for a = 0:0.25:1
    r = spline_point_3d(spline, a);
    dr = spline_dot_point_3d(spline, a);
    ndr = dr / norm(dr);
    ndrh = [ndr(1); ndr(2)];
    
    x = 1;
    y = 1;
    if (ndrh(1) > eps)
        y = -ndrh(1)/ndrh(2);
    else
        x = -ndrh(2)/ndrh(1);
    end
    vn = [x;y;0];
    vn = vn / norm(vn);
    
    curve_center_1 = r + d_crv*vn;
    curve_center_2 = r + d_crv*vn;
    for k = 1:size(obstacles, 2)
        dr1 = obstacles(:,k) - curve_center_1;
        dr2 = obstacles(:,k) - curve_center_2;
        if (norm(dr1) < d_crv + d_obs) & (norm(dr2) < d_crv + d_obs)
            plot3(r(1) - d_crv*vn(1) + xs_crv, r(2) - d_crv*vn(2) + ys_crv, r(3) - d_crv*vn(3) + zs_crv, 'c')
            plot3(r(1) + d_crv*vn(1) + xs_crv, r(2) + d_crv*vn(2) + ys_crv, r(3) + d_crv*vn(3) + zs_crv, 'm')
            
            plot3([r(1) - d_crv*ndr(1), r(1) + d_crv*ndr(1)], [r(2) - d_crv*ndr(2), r(2) + d_crv*ndr(2)], [r(3) - d_crv*ndr(3), r(3) + d_crv*ndr(3)], 'g')
            plot3([r(1) - d_crv*vn(1), r(1) + d_crv*vn(1)], [r(2) - d_crv*vn(2), r(2) + d_crv*vn(2)], [r(3) - d_crv*vn(3), r(3) + d_crv*vn(3)], 'r')
        end
    end
    
end

end