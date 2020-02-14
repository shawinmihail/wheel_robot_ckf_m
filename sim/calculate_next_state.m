function [next_state, next_yaw] = calculate_next_state(state, yaw, ctrl, grad_surf_function, dt)
    
    %% update state
    % state = [rI vI qIB]
    r0 = state(1:3);
    v0 = state(4:6);
    q0 = state(7:10);
    yaw0 = yaw;
    
    % ctrl = [v u]
    v = ctrl(1);
    u = ctrl(2);
    
    
    %% r1
    r1 = r0 + v0 * dt;
    
    %% n_unit0
    n_unit0 = grad_surf_function(r0);
    n_unit0 = n_unit0 / norm(n_unit0);
        
    %% n_unit1
    n_unit1 = grad_surf_function(r1);
    n_unit1 = n_unit1 / norm(n_unit1);
    
    %% q1
    q_surf1 = calc_q_surf(n_unit1);
    yaw1 = yaw0 + v * u * dt;
    q1 = calc_q_full(grad_surf_function, r1, yaw1);
    next_yaw = yaw1;
    
    %% v1
    v1 = quatRotate(q1, [v;0;0]);
    
    next_state = [r1;v1;q1];
end

