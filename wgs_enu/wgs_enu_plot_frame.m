function wgs_enu_plot_frame(r, R, m)
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

ex = R*ex*m;
ey = R*ey*m;
ez = R*ez*m;


green = [0.1 0.5 0.1];
plot3([r(1) r(1) + ex(1)], [r(2) r(2) +  ex(2)], [r(3) r(3) + ex(3)], 'Color', 'r', 'LineWidth', 2)
plot3([r(1) r(1) + ey(1)], [r(2) r(2) +  ey(2)], [r(3) r(3) + ey(3)], 'Color', green, 'LineWidth', 2)
plot3([r(1) r(1) + ez(1)], [r(2) r(2) +  ez(2)], [r(3) r(3) + ez(3)], 'Color', 'b', 'LineWidth', 2)

end