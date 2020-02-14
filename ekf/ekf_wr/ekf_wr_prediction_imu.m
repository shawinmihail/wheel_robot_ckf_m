function [X, sqrtP] = ekf_wr_prediction_imu(X, sqrtP, sqrtQ, a_mes, w_mes, dt)
% X [r v q]

%% X
g = [0;0;-10];
q = X(7:10);
r_dot = X(4:6);
v_dot = quatRotate(q, a_mes) + g;

qw = [0;w_mes];
q_dot = 0.5 * quatMultiply(q, qw);

X_dot = [r_dot; v_dot; q_dot];
X = X + X_dot * dt;
X(7:10) = X(7:10) / norm(X(7:10));

%% P
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
E33 = eye(3, 3);

% v_dot = quatRotate(q, a_mes) + g;
Mvq = quat_rot_jacob(q(1),q(2),q(3),q(4),a_mes(1),a_mes(2),a_mes(3));
% q_dot = 0.5 * quatMultiply(q, qw);
Mqq = poison_eq_jacob(q(1),q(2),q(3),q(4),w_mes(1),w_mes(2),w_mes(3));


%%
F = [O33 E33 O34; 
     O33 O33 Mvq;
     O43 O43 Mqq];
 
I = eye(10);
sqrtP = tria([(I+F*dt) * sqrtP, sqrtQ], 10);

end

