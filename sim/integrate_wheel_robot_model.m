function [next_state, next_state_dot, next_ctrl] = integrate_wheel_robot_model(curr_state, curr_state_dot, curr_ctrl, next_ctrl, dt)

    %% process ctrl input
    % ctrl = [v, u]
	Kv = 1.0;
	Ku = 1.0;
	dv = next_ctrl(1) - curr_ctrl(1);
	du = next_ctrl(2) - curr_ctrl(2);

	next_ctrl(1) = curr_ctrl(1) + Kv * dv * dt;
	next_ctrl(2) = curr_ctrl(2) + Ku * du * dt;
    
    %% update state
    % state = [x y phi]
    next_state = curr_state + curr_state_dot * dt;
    
    %% update state dot
    x_dot = next_ctrl(1) * cos(next_state(3));
	y_dot = next_ctrl(1) * sin(next_state(3));
    phi_dot = next_ctrl(1) * next_ctrl(2);
    next_state_dot = [x_dot; y_dot; phi_dot];
    

end

