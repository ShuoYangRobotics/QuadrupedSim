function quad_optimal_transition_print_times(opt_state, param)

sp = TransitionStateProcessor(param);

time_list = [];
for i = 1:param.leg_num
    for j = 1:param.st_knot_num*2
        time = sp.getLegStanceTime(opt_state,i,j,1);
        time_list = [time_list time];
    end
    time_list
    % plot phase time 
    phase_time = sp.getPhaseTime(opt_state,i,1)
    time_list = [];
end
end