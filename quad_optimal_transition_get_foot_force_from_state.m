function F = quad_optimal_transition_get_foot_force_from_state(state, param)

% stance_state to foot force, just some indexing 
F = zeros(3,4);
F(:,1) = state(19:21)';
F(:,2) = state(22:24)';
F(:,3) = state(25:27)';
F(:,4) = state(28:30)';


end