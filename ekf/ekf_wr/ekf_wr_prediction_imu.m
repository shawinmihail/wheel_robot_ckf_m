function [X, sqrtP] = ekf_wr_prediction_imu(X, sqrtP, sqrtQ, a_mes, w_mes, imu_attachment_r, dt)

%% X [r v a q w]
g = [0;0;-10];
r0 = X(1:3);
v0 = X(4:6);
q0 = X(10:13);
w0 = w_mes;
a0 = quatRotate(q0, a_mes + cross(w0, cross(imu_attachment_r,w0))) + g;


%% X next
qw0 = [0;w0];
r_next = r0 + v0 * dt;
v_next = v0 + a0 * dt;
a_next = a0;
q_next = q0 +  0.5 * quatMultiply(q0, qw0) * dt;
% q_next = q_next / norm(q_next);
w_next = w0;

X = [r_next; v_next; a_next; q_next; w_next];

%% P
O33 = zeros(3, 3);
O34 = zeros(3, 4);
O43 = zeros(4, 3);
E33 = eye(3, 3);

Maq = M_rddot_dq_fcn(...
    q0(1),q0(2),q0(3),q0(4),a_mes(1),a_mes(2),a_mes(3),imu_attachment_r(1),imu_attachment_r(2),imu_attachment_r(3),w_mes(1),w_mes(2),w_mes(3));
Maw = M_rddot_dw_fcn(...
    w0(1),w0(2),w0(3),q0(1),q0(2),q0(3),q0(4),imu_attachment_r(1),imu_attachment_r(2),imu_attachment_r(3));
Mqq = M_qdot_dq_fcn(q0(1),q0(2),q0(3),q0(4),w0(1),w0(2),w0(3));
Mqw = M_qdot_dw_fcn(w0(1),w0(2),w0(3),q0(1),q0(2),q0(3),q0(4));
%%
F = [O33 E33 O33 O34 O33; 
     O33 O33 E33 Maq Maw;
     O33 O33 O33 O34 O33;
     O43 O43 O43 Mqq Mqw
     O33 O33 O33 O34 O33];
 
I = eye(16);
sqrtP = tria([(I+F*dt) * sqrtP, sqrtQ], 16);

end

