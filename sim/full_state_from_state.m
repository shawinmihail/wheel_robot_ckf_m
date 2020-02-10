function full_state = full_state_from_state(state, state_dot, last_state_dot, dt)
    
    full_state = zeros(10, 1);
	%% r
	full_state(1) = state(1);
	full_state(2) = state(2);
% 	full_state(3) = 0;
    
	%% v
	full_state(4) = state_dot(1);
	full_state(5) = state_dot(2);
% 	full_state(6) = 0;
    
	%% a
	full_state(7) = (state_dot(1) - last_state_dot(1)) / dt;
	full_state(8) = (state_dot(2) - last_state_dot(2)) / dt;
% 	full_state(9) = 0;
    
	%% q
	full_state(10:13) = quatFromEul([0;0;state(3)]);

    
	%% w
% 	full_state(14) = 0;
%   full_state(15) = 0;
    full_state(16) = state_dot(3);
end

