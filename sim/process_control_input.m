function next_ctrl = process_control_input(curr_ctrl, next_ctrl, dt)

%% actuators model
% ctrl = [v, u]
Kv = 1.0;
Ku = 1.0;
dv = next_ctrl(1) - curr_ctrl(1);
du = next_ctrl(2) - curr_ctrl(2);

next_ctrl(1) = curr_ctrl(1) + Kv * dv * dt;
next_ctrl(2) = curr_ctrl(2) + Ku * du * dt;