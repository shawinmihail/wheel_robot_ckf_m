function curr_full_state = calculate_state_derivatives(curr_full_state, dt)
    % full state = [r v a q w wdot]
    a = (next_state(4:6) - curr_state(4:6))/ dt;
    q_dot = (next_state(7:10) - curr_state(7:10))/ dt;
    qw = 2 * quatMultiply(quatDual(curr_state(7:10)), q_dot);
    w = qw(2:4);
    
    curr_full_state
end

