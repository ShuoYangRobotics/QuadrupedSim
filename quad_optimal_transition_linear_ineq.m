function [Aineq, bineq] = quad_optimal_transition_linear_ineq(state, robot_state1, robot_state2, param)
% revised on 5/24/2020
% linear inequality constraints limits time of knots to lie within bounds
% also variable bounds are included
sp = TransitionStateProcessor(param);

time_margin = 0.001; % very important!!!!!!
phase_margin = param.total_time/4.3; % very important!!!!!!

c_phase = param.leg_num;   % phase(1) + p_margin < phase(2)  -->  phase(1)- phase(2)  < -p_margin

c_leg_swing = param.sw_knot_num+1;  % phase(1) < t1            -->  phase(1) - t1 < -t_margin
                                    % t1 < t2 - t_margin      -->        t1 - t2 < -t_margin
                                    % tn < phase(2)+t_margin  -->  tn - phase(2) < -t_margin
                                    
c_leg_stance = 2*(param.st_knot_num+1); % 0 < t1              -->         0 - t1 < 0
                                        % t1 < t2 - t_margin  -->        t1 - t2 < -t_margin
                                        % tn < phase(1)+t_margin       -->  tn - phase(1) < -t_margin
                                        % phase(2) < t1       -->  phase(2) - t1 < -t_margin
                                        % t1 < t2 - t_margin  -->        t1 - t2 < -t_margin
                                        % tn < total_time     -->  tn - total_time < 0
c_total = c_phase + c_leg_swing*param.leg_num + c_leg_stance*param.leg_num;                            

Aineq = spalloc(c_total, param.opt_state_size, 63);  
bineq = zeros(c_total,1);

c_start_idx = 0;

%% phase time constraint 
for i = 1:param.leg_num 
    c_start_idx = c_start_idx + 1;
    knot_idx = sp.getPhaseTime(state, i, 0);
    Aineq(c_start_idx, knot_idx(1)) =  1;   %tb-ta > phase_margin
    Aineq(c_start_idx, knot_idx(2)) = -1;
    bineq(c_start_idx) = - phase_margin;
end

%% swing time constraint 
for i = 1:param.leg_num 
    phase_knot_idx = sp.getPhaseTime(state, i, 0);
    % swing 
    for j = 1:param.sw_knot_num+1
        c_start_idx = c_start_idx + 1;
        if (j == 1)
            swing_knot_idx = sp.getLegSwingTime(state, i, j ,0);
            Aineq(c_start_idx, phase_knot_idx(1)) =  1;
            Aineq(c_start_idx, swing_knot_idx)    = -1;
            bineq(c_start_idx) = -time_margin;                       %  phase(1) - t1 < -t_margin           
        elseif (j == param.sw_knot_num+1)
            swing_knot_idx = sp.getLegSwingTime(state, i, j-1 ,0);
            Aineq(c_start_idx, swing_knot_idx) = 1;
            Aineq(c_start_idx, phase_knot_idx(2)) = -1;
            bineq(c_start_idx) = -time_margin;                        %  tn - phase(2) < -t_margin
        else
            swing_knot_idx1 = sp.getLegSwingTime(state, i, j-1 ,0);
            swing_knot_idx2 = sp.getLegSwingTime(state, i, j ,0);
            Aineq(c_start_idx, swing_knot_idx1) =  1;
            Aineq(c_start_idx, swing_knot_idx2) = -1;
            bineq(c_start_idx) = -time_margin;                       %  t1 - t2 < -t_margin
        end
    end
end
%% stance time constraint 
for i = 1:param.leg_num 
    phase_knot_idx = sp.getPhaseTime(state, i, 0);
    % stance part 1 
    for j = 1:param.st_knot_num+1
        c_start_idx = c_start_idx + 1;
        if (j == 1)
            swing_knot_idx = sp.getLegStanceTime(state, i, j ,0);
            Aineq(c_start_idx, swing_knot_idx)    = -1;
            bineq(c_start_idx) = -time_margin;                                  %  0 - t1 < -time_margin
        elseif (j == param.st_knot_num+1)
            swing_knot_idx = sp.getLegStanceTime(state, i, j-1 ,0);
            Aineq(c_start_idx, swing_knot_idx) = 1;
            Aineq(c_start_idx, phase_knot_idx(1)) = -1;
            bineq(c_start_idx) = -time_margin;                          %  tn - phase(1) < -t_margin
        else
            swing_knot_idx1 = sp.getLegStanceTime(state, i, j-1 ,0);
            swing_knot_idx2 = sp.getLegStanceTime(state, i, j ,0);
            Aineq(c_start_idx, swing_knot_idx1) =  1;
            Aineq(c_start_idx, swing_knot_idx2) = -1;
            bineq(c_start_idx) = -time_margin;                       %  t1 - t2 < -t_margin
        end
    end
    % stance part 2 
    for j = 1:param.st_knot_num+1
        c_start_idx = c_start_idx + 1;
        if (j == 1)
            swing_knot_idx = sp.getLegStanceTime(state, i, param.st_knot_num+j ,0);
            Aineq(c_start_idx, phase_knot_idx(2)) =  1;
            Aineq(c_start_idx, swing_knot_idx)    = -1;
            bineq(c_start_idx) = -time_margin;                       %  phase(2) - t1 < -time_margin
        elseif (j == param.st_knot_num+1)
            swing_knot_idx = sp.getLegStanceTime(state, i, param.st_knot_num+j-1 ,0);
            Aineq(c_start_idx, swing_knot_idx) = 1;
            bineq(c_start_idx) = param.total_time-time_margin;                                  %  tn - total < -time_margin
        else
            swing_knot_idx1 = sp.getLegStanceTime(state, i, param.st_knot_num+j-1 ,0);
            swing_knot_idx2 = sp.getLegStanceTime(state, i, param.st_knot_num+j ,0);
            Aineq(c_start_idx, swing_knot_idx1) =  1;
            Aineq(c_start_idx, swing_knot_idx2) = -1;
            bineq(c_start_idx) = -time_margin;                       %  t1 - t2 < -t_margin
        end
    end
end

%% variable bounds 
% [lb, ub] = quad_flat_optimal_transition_state_bound(robot_state1, robot_state2,  param);
% Aineq(c_total+1:c_total+param.opt_state_size,:) = -speye(param.opt_state_size);
% bineq(c_total+1:c_total+param.opt_state_size) = -lb;
% Aineq(c_total+1+param.opt_state_size:c_total+param.opt_state_size*2,:) = speye(param.opt_state_size);
% bineq(c_total+1+param.opt_state_size:c_total+param.opt_state_size*2) = ub;
end