function [a, w] = calculate_acc_rotrate(curr_state, next_state, dt)
    a = (next_state(4:6) - curr_state(4:6))/ dt;
    q_dot = (next_state(7:10) - curr_state(7:10))/ dt;
    qw = 2 * quatMultiply(quatDual(curr_state(7:10)), q_dot);
    w = qw(2:4);
end

