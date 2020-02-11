function [next_state, next_state_dot, next_ctrl] = integrate_wheel_robot_model(curr_state, curr_state_dot, next_ctrl, dt)
    

    
    %% update state
    % state = [x y phi]
    next_state = curr_state + curr_state_dot * dt;
    
    %% update state dot
    x_dot = next_ctrl(1) * cos(next_state(3));
	y_dot = next_ctrl(1) * sin(next_state(3));
    phi_dot = next_ctrl(1) * next_ctrl(2);
    next_state_dot = [x_dot; y_dot; phi_dot];

end

