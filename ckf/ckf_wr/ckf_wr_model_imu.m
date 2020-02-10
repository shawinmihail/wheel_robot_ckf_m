function X_dot = ckf_wr_model_imu(X, a, w)

% X = [r v q]

r = X(1:3);
v = X(4:6);
q = X(7:10);

r_dot = v;
v_dot = a;

qw = [0;w];
q_dot = 0.5*quatMultiply(q, qw);
X_dot = [r_dot; v_dot; q_dot];
end

