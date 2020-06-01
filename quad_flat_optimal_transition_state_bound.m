function [lb, ub] = quad_flat_optimal_transition_state_bound(robot_state1, robot_state2,  param)
sp = TransitionStateProcessor(param);
idx_state = 1:param.opt_state_size;
% implemented on Apr 18, 2020, revised on May 24, 2020
lb = -ones(param.opt_state_size,1)*inf;
ub = ones(param.opt_state_size,1)*inf;

%% time bounds, may be dupliated with Ax<b

for i = 1:param.leg_num 
    % phase time
    knot_idx = sp.getPhaseTime(idx_state, i, 0);
    lb(knot_idx) = 0;
    ub(knot_idx) = param.total_time;
    
    % swing knot time
    for j = 1:param.sw_knot_num
        knot_idx = sp.getLegSwingTime(idx_state, i, j, 0);
        lb(knot_idx) = 0;
        ub(knot_idx) = param.total_time;
    end
    
    % stance knot time
    for j = 1:2*param.st_knot_num
        knot_idx = sp.getLegStanceTime(idx_state, i, j, 0);
        lb(knot_idx) = 0;
        ub(knot_idx) = param.total_time;
    end
end
% com pos time
for i = 1:param.co_knot_num
knot_idx = sp.getComPosTime(idx_state,i, 0);
lb(knot_idx) = 0;
ub(knot_idx) = param.total_time;
% com pitch time
knot_idx = sp.getComAngTime(idx_state,i, 0);
lb(knot_idx) = 0;
ub(knot_idx) = param.total_time;
end

%% com pos knot
com_start = robot_state1(1:3);
com_end = robot_state2(1:3);
max_com_pos = max(com_start, com_end);
min_com_pos = min(com_start, com_end);
for i = 1:param.co_knot_num
    knot_idx = sp.getComPosKnot(idx_state, i, 0);        
    com_lb = min_com_pos - param.com_bound;
    com_ub = max_com_pos + param.com_bound;
    Cvel = param.max_com_pos_vel;
    lb(knot_idx) = [com_lb;-Cvel;-Cvel;-Cvel];
    ub(knot_idx) = [com_ub; Cvel; Cvel; Cvel];
end
%% com angle knot
com_angle_start = robot_state1(4:6);
com_angle_end = robot_state2(4:6);
average_com_pitch = (com_angle_start+com_angle_end)/2;
for i = 1:param.co_knot_num
    knot_idx = sp.getComAngKnot(idx_state, i, 0);        
    com_lb = average_com_pitch - param.com_pitch_bound;
    com_ub = average_com_pitch + param.com_pitch_bound;
    Cvel = param.max_com_pos_vel;
    lb(knot_idx) = [com_lb;-Cvel;-Cvel;-Cvel];
    ub(knot_idx) = [com_ub; Cvel; Cvel; Cvel];
end

%% swing knots
p_e_start = quad_optimal_transition_get_foot_pos_from_state(robot_state1, param);
p_e_end = quad_optimal_transition_get_foot_pos_from_state(robot_state2, param);
for i = 1:param.leg_num
    averge_pos1 = (0.51*p_e_start(:,i)+0.49*p_e_end(:,i))+[0;0;param.swing_height];
    averge_pos2 = (0.49*p_e_start(:,i)+0.51*p_e_end(:,i))+[0;0;param.swing_height];
    averge_pos = [averge_pos1 averge_pos2];
    for j = 1:param.sw_knot_num
        knot_idx = sp.getLegSwingKnot(idx_state, i, j, 0);
        Pvel = param.max_foot_pos_vel;
        swing_lb = averge_pos(:,j) - param.swing_bound;
        swing_ub = averge_pos(:,j) + param.swing_bound;
        lb(knot_idx) = [swing_lb;-Pvel;-Pvel;-Pvel];
        ub(knot_idx) = [swing_ub; Pvel; Pvel; Pvel];
    end
end

%% stance force box constraint
for i = 1:param.leg_num 
    for j = 1:2*param.st_knot_num
        knot_idx = sp.getLegStanceKnot(idx_state, i, j, 0);
        F1 = param.terrain_u2*param.weight_ratio*param.total_mass*param.g/100;
        F2 = param.terrain_u2*param.weight_ratio*param.total_mass*param.g/100;
        F3 = param.weight_ratio*param.total_mass*param.g/100;
        Fvel = param.max_foot_force_vel;
        lb(knot_idx) = [-F1;-F2;  0;-Fvel;-Fvel;-Fvel];
        ub(knot_idx) = [ F1; F2; F3; Fvel; Fvel; Fvel];
    end
end

end
