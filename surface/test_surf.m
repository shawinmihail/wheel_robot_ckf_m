clc
clear


%% init
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

%% surf
surf_function = @(x,y)(0.5 * x + 0.1 * y);
grad_surf_function = @(x,y)[-0.5 ; -0.1; 1];


figure
hold on
grid on

x = 1;
y = 1;
z = surf_function(x, y);
yaw = 0;

n_unit = grad_surf_function(x, y);
n_unit = n_unit / norm(n_unit);
q_surf = calc_q_surf(n_unit);
q_yaw = calc_q_yaw(yaw, n_unit);
q = quatMultiply(q_yaw, q_surf);

dt = 0.01;
for i = 1:10
    v = 1;
    u = 0;
    
    %% point
    rdot = quatRotate(q, [v;0;0]);
    yaw_dot = v * u;
    
    r_next = r + rdot * dt;
    n_unit_next = grad_surf_function(r_next(1), r_next(2));
    n_unit_next = n_unit_next / norm(n_unit_next);
    
    n_dot = (n_unit_next - n_unit) / dt;
    
    %% plot
    [X,Y] = meshgrid(0:0.1:10);  
    Z = surf_function(X,Y);
    gradZ = grad_surf_function(x, y);


    surf(X,Y,Z, 'FaceAlpha',0.5, 'EdgeColor', 'None')
    plot3([x x + exB(1)], [y y +  exB(2)], [z z + exB(3)], 'Color', 'r', 'LineWidth', 2)
    plot3([x x + eyB(1)], [y y +  eyB(2)], [z z + eyB(3)], 'Color', 'g', 'LineWidth', 2)
    plot3([x x + ezB(1)], [y y +  ezB(2)], [z z + ezB(3)], 'Color', 'b', 'LineWidth', 2)

    xlim([0 5])
    ylim([0 5])
    zlim([0 5])
end