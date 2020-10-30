function [X, sqrtP] = ekf3_wr_predict(X, sqrtP, Q, dt)

%% X [r v a q w]
r0 = X(1:3);
v0 = X(4:6);
a0 = X(7:9);
q0 = X(10:13);
w0 = X(14:16);

%% X next
r_next = r0 + v0 * dt;
v_next = v0 + a0 * dt;
v_next = v0 + cross(w0, v0)*dt;
a_next = a0;
q_next = q0 +  0.5 * quatMultiply(q0, [0;w0]) * dt;
q_next = q_next / norm(q_next);
w_next = w0;
X = [r_next; v_next; a_next; q_next; w_next];

%% P
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
E33 = eye(3, 3);

Mqq = M_qdot_dq_fcn(q0(1),q0(2),q0(3),q0(4),w0(1),w0(2),w0(3));
Mqw = M_qdot_dw_fcn(w0(1),w0(2),w0(3),q0(1),q0(2),q0(3),q0(4));

%%
F = [O33 E33 O33 O34 O33; 
     O33 O33 E33 O34 O33;
     O33 O33 O33 O34 O33;
     O43 O43 O43 Mqq Mqw
     O33 O33 O33 O34 O33];
 
I = eye(16);
sqrtP = tria([(I+F*dt) * sqrtP, Q * sqrt(dt)], 16);
end

