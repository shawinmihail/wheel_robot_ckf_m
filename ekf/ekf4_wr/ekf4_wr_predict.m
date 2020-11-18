function [X, P] = ekf4_wr_predict(X, a_imu, w_imu, P, Q, dt, g)

%% X [r v a q w]
r0 = X(1:3);
v0 = X(4:6);
q0 = X(7:10);

a0 = quatRotate(q0, a_imu) + g;
w0 = w_imu;

%% X next
r_next = r0 + v0 * dt;
% v_next = v0 + a0 * dt;
v_next = v0 + cross(quatRotate(q0, w0), v0)*dt;
q_next = q0 +  0.5 * quatMultiply(q0, [0;w0]) * dt;
q_next = q_next / norm(q_next);
X = [r_next; v_next; q_next];

%% P
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
E33 = eye(3, 3);

Mqq = M_qdot_dq_fcn(q0(1),q0(2),q0(3),q0(4),w0(1),w0(2),w0(3));
Mqq = Mqq(2:end, 2:end);
Mvq = -x_operator(v0) * quat_rot_fcn_j(q0(1),q0(2),q0(3),q0(4),w0(1),w0(2),w0(3));
Mvq = Mvq(:, 2:end);

%%
F = [O33 E33 O33; 
     O33 O33 Mvq;
     O33 O33 Mqq];

I = eye(9);
PHI = I + F * dt;
P = PHI * P * PHI + Q;
end

