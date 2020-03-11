function [X, sqrtP] = ekf2_wr_prediction_imu(X, sqrtP, Q, a_mes, w_mes, dt)

N = length(X);
%% X [r v a q]
g = [0;0;-10];
r0 = X(1:3);
v0 = X(4:6);
q0 = X(10:13);
a0 = quatRotate(q0, a_mes) + g;

%% X next
qw0 = [0;w_mes];
r_next = r0 + v0 * dt;
v_next = v0 + a0 * dt;
a_next = a0;
q_next = q0 +  0.5 * quatMultiply(q0, qw0) * dt;
q_next = q_next / norm(q_next);
X = [r_next; v_next; a_next; q_next];

%% P
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
E33 = eye(3, 3);

Maq = M_rddot_dq_fcn(q0(1),q0(2),q0(3),q0(4),a0(1),a0(2),a0(3));
Mqq = M_qdot_dq_fcn(q0(1),q0(2),q0(3),q0(4),w_mes(1),w_mes(2),w_mes(3));
%%
F = [O33 E33 O33 O34; 
     O33 O33 O33 Maq;
     O33 O33 O33 O34
     O43 O43 O43 Mqq];
 
I = eye(N);
sqrtP = tria([(I+F*dt) * sqrtP, Q * sqrt(dt)], N);
end

