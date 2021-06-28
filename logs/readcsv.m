clc
clear
close all
figure
hold on
grid on

tb = readtable('logs/routes/path_june1_along_car1.csv');
set = tb{:,:};
set = set';

[splines] = M_spline_from_set(set);

splines_length = length(splines);
for i = 1:splines_length

spline = splines(:,:,i);

for a = 0:0.33:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'r.')
end

end