clc
clear
close all

tb = readtable('car2.csv');
set = tb{1:25,1:end};
set = set';
[splines] = M_spline_from_set(set); 

%%
maneur_displacement = 0.1;
maneur_radius = 3;

vp_dir = set(:,1) - set(:,2);
vp_dir = vp_dir / norm(vp_dir);
p1 = set(:,1) + vp_dir * maneur_displacement;
p2 = p1 + vp_dir * maneur_radius;
p3 = p2 + vp_dir * maneur_radius;
virtual_set = [p3 p2 p1 set(:,1)];
virtual_way = M_spline_from_set(virtual_set);


figure
hold on
grid on
splines_s = size(splines);
for i = 1:splines_s(3)

spline = splines(:,:,i);
point = spline * [1; 0; 0; 0];
plot3(point(1), point(2), point(3), 'g*')

for a = 0:0.1:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'r.')
end

end

% plot3(p1(1), p1(2), p1(3), 'k*')

splines_s = size(virtual_way);
for i = 1:splines_s(3)

spline = virtual_way(:,:,i);
point = spline * [1; 0; 0; 0];
plot3(point(1), point(2), point(3), 'k*')

for a = 0:0.1:1
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'k.')
end

end