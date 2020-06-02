function state_init = quad_optimal_transition_init_state(param, robot_state1, robot_state2, terrain_heights)
% notice that at this stage, robot_state1 and robot_state2 are scaled
sp = TransitionStateProcessor(param);

state_init = zeros(param.opt_state_size,1);

% 2-2
state_init = sp.setPhaseTime(state_init, [0.07;0.47]*param.total_time,1);     
state_init = sp.setPhaseTime(state_init, [0.52;0.96]*param.total_time,2);       
state_init = sp.setPhaseTime(state_init, [0.52;0.96]*param.total_time,3);     
state_init = sp.setPhaseTime(state_init, [0.07;0.47]*param.total_time,4);         
% 1-1-1-1
% state_init = sp.setPhaseTime(state_init, [0.01;0.25]*param.total_time,1);     
% state_init = sp.setPhaseTime(state_init, [0.26;0.50]*param.total_time,2);       
% state_init = sp.setPhaseTime(state_init, [0.51;0.75]*param.total_time,3);     
% state_init = sp.setPhaseTime(state_init, [0.76;0.99]*param.total_time,4);           



%% start and final poses, notice they are scaled
com_pos_start = robot_state1(1:3);
com_pos_end = robot_state2(1:3);
com_angle_start = robot_state1(4:6);
com_angle_end = robot_state2(4:6);

% convert joint pose in robot_states into earth foot pose
p_e_start = quad_optimal_transition_get_foot_pos_from_state(robot_state1, param);
p_e_end = quad_optimal_transition_get_foot_pos_from_state(robot_state2, param);
% get foot force in earth frame
F_e_start = quad_optimal_transition_get_foot_force_from_state(robot_state1, param);
F_e_end = quad_optimal_transition_get_foot_force_from_state(robot_state2, param);

%% init swing knots
for i=1:param.leg_num
    phase_t = sp.getPhaseTime(state_init, i, 1);  % last 1 means get value
    swing_duration = phase_t(2) - phase_t(1); 
    for j =1:param.sw_knot_num
        frac = 0.4+(j-1)*0.2;
        [init_swing_pos,init_swing_vel,~] = hermite_cubic_knot(frac*swing_duration,...
            0, p_e_start(:,i), [0;0;0], swing_duration, p_e_end(:,i), [0;0;0]); 
        
        knot_time = phase_t(1)+frac*swing_duration;  
        % time in phase (0,1)
        knot_val = [...
           init_swing_pos(:)+[0;0;param.swing_height];
           init_swing_vel(:)...
        ];
        state_init = sp.setLegSwingTime(state_init, knot_time, i, j);
        state_init = sp.setLegSwingKnot(state_init, knot_val, i, j);
    end    
end

%% init stance knots
for i=1:param.leg_num
    phase_t = sp.getPhaseTime(state_init, i, 1);
    duration1 = phase_t(1);                  % ta-0
    duration2 = param.total_time - phase_t(2);  % T-tb

    for j=1:param.st_knot_num
        frac = j/(param.st_knot_num+1);
        init_stance_knots=[F_e_start(:,i); [0;0;0]];
        state_init = sp.setLegStanceTime(state_init, frac*duration1, i, j);
        state_init = sp.setLegStanceKnot(state_init, init_stance_knots, i, j);
    end
    for j=1:param.st_knot_num
        frac = j/(param.st_knot_num+1);
        init_stance_knots=[F_e_end(:,i); [0;0;0]];
        state_init = sp.setLegStanceTime(state_init, phase_t(2)+frac*duration2, i, param.st_knot_num+j);
        state_init = sp.setLegStanceKnot(state_init, init_stance_knots, i, param.st_knot_num+j);
    end
end

%% init COM related 

for j =1:param.co_knot_num
    frac = j/(param.co_knot_num+1);
    [com_knot_pos, com_knot_vel, ~] = hermite_cubic_knot(frac*param.total_time,...
    0, com_pos_start, [0;0;0], param.total_time, com_pos_end, [0;0;0]); 
    [com_angle_pos, com_angle_vel, ~] = hermite_cubic_knot(frac*param.total_time,...
    0, com_angle_start, 0, param.total_time, com_angle_end, 0); 

    state_init = sp.setComPosTime(state_init, j, frac*param.total_time);
    state_init = sp.setComPosKnot(state_init, [com_knot_pos;com_knot_vel],j);
    
    
    state_init = sp.setComAngTime(state_init, j, frac*param.total_time);
    state_init = sp.setComAngKnot(state_init, [com_angle_pos;com_angle_vel],j);
end

% state_init = state_init + 0.0001*randn(param.opt_state_size,1);
end