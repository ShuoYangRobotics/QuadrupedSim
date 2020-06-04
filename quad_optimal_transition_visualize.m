function quad_optimal_transition_visualize(fig_id, vis_state, robot_state1, robot_state2, ref_com_pos_start, param)
% vis_state = opt_state_init;
% vis_state = opt_state_soln;
% notice state_soln is based on scaled robot_state1
addpath('../CasADi')
import casadi.*

load('casadi_state_derive_quad','casadi_sx_com_angle_func',...
                               'casadi_sx_com_pos_func',...
                               'casadi_sx_list_force_func',...
                               'casadi_sx_list_swing_pos_func');
% calculate necessary start and end pos and force 
% the actual pos of p_e and com_pos need to add this reference value to
% scale back

com_pos_start = robot_state1(1:3);
com_pos_end = robot_state2(1:3);
com_ang_start = robot_state1(4:6);
com_ang_end = robot_state2(4:6);
p_e_start = quad_optimal_transition_get_foot_pos_from_state(robot_state1, param);
p_e_end = quad_optimal_transition_get_foot_pos_from_state(robot_state2, param);
F_e_start = quad_optimal_transition_get_foot_force_from_state(robot_state1, param);
F_e_end = quad_optimal_transition_get_foot_force_from_state(robot_state2, param);

t_list = 0:0.01:param.total_time;  
com_pos_list = zeros(3,size(t_list,2));
com_angle_list = zeros(3,size(t_list,2));
foot_pos_list = cell(param.leg_num,1); 
for i=1:param.leg_num foot_pos_list{i} = zeros(3,size(t_list,2)); end
foot_force_list = cell(param.leg_num,1); 
for i=1:param.leg_num foot_force_list{i} = zeros(3,size(t_list,2)); end

for i=1:size(t_list,2)
    cur_time = t_list(i);
    angle_val = casadi_sx_com_angle_func(cur_time, vis_state, com_ang_start, com_ang_end);
    com_angle_list(:,i) = full(angle_val);
    
    com_pos_val = casadi_sx_com_pos_func(cur_time, vis_state, com_pos_start, com_pos_end);
    com_pos_list(:,i) = full(com_pos_val) + ref_com_pos_start;
    
    for j = 1:param.leg_num
        foot_pos_val = casadi_sx_list_swing_pos_func{j}(cur_time, vis_state, p_e_start(:,j), p_e_end(:,j));
        foot_pos_list{j}(:,i) = full(foot_pos_val) + ref_com_pos_start;
    end
    % force is more complicated
    for j = 1:param.leg_num
        foot_force_val = casadi_sx_list_force_func{j}(cur_time, vis_state, F_e_start(:,j), F_e_end(:,j));
        foot_force_list{j}(:,i) = full(foot_force_val)*param.force_scale;
    end
end

sp = TransitionStateProcessor(param);
figure(fig_id)
set(gcf, 'units','normalized','outerposition',[0.3+0.3*(fig_id-30) 0.2 0.3 0.7]);
clf; hold on;

subplot_num = 1+param.leg_num;
subplot(subplot_num,2,1);
% plot com angle curve
plot(t_list, com_angle_list(1,:)); hold on;
plot(t_list, com_angle_list(2,:)); hold on;
plot(t_list, com_angle_list(3,:)); hold on;
% plot com angle knot
for i= 1:param.co_knot_num
time = sp.getComAngTime(vis_state,i,1);
val = sp.getComAngKnot(vis_state,i,1);
plot(time, val(1),'ro'); hold on;
end


title(strcat('com angle'));
subplot(subplot_num,2,2);
plot(t_list, com_pos_list(1,:)); hold on;
plot(t_list, com_pos_list(2,:)); hold on;
plot(t_list, com_pos_list(3,:)); hold on;
% plot com pos knot, remember to scale back
for i= 1:param.co_knot_num
time = sp.getComPosTime(vis_state,i,1);
val = sp.getComPosKnot(vis_state,i,1);
val = val(1:3) + ref_com_pos_start;
plot(time, val(1),'ro'); hold on;
plot(time, val(2),'ro'); plot(time, val(3),'ro');
end
title(strcat('com position'));

for i = 1:param.leg_num
    subplot(subplot_num,2,1+(i-1)*2+2);
    plot(t_list, foot_pos_list{i}(1,:)); hold on;
    plot(t_list, foot_pos_list{i}(2,:)); hold on;
    plot(t_list, foot_pos_list{i}(3,:)); hold on;
    % plot foot pos knot, remember to scale back
    for j = 1:param.sw_knot_num
        time = sp.getLegSwingTime(vis_state,i,j,1);
        val = sp.getLegSwingKnot(vis_state,i,j,1);
        val = val(1:3) + ref_com_pos_start;
%         plot(time, val(1),'ro'); 
%         plot(time, val(2),'ro'); 
        plot(time, val(3),'ro');
    end
    % plot phase time 
    phase_time = sp.getPhaseTime(vis_state,i,1);
    y1 = ylim;
    line([phase_time(1) phase_time(1)],[y1(1) y1(2)],'Color','black','LineStyle','--'); hold on;
    line([phase_time(2) phase_time(2)],[y1(1) y1(2)],'Color','black','LineStyle','--'); hold on;
    title(strcat('leg', int2str(i), ' pos'));
end
for i = 1:param.leg_num
    subplot(subplot_num,2,(i)*2+2);
    plot(t_list, foot_force_list{i}(1,:)); hold on;
    plot(t_list, foot_force_list{i}(2,:)); hold on;
    plot(t_list, foot_force_list{i}(3,:)); hold on;
    % plot foot force knot, remember to scale back
    for j = 1:param.st_knot_num*2
        time = sp.getLegStanceTime(vis_state,i,j,1);
        val = sp.getLegStanceKnot(vis_state,i,j,1);
        val = val(1:3)*param.force_scale;
%         plot(time, val(1),'ro'); 
%         plot(time, val(2),'ro'); 
        plot(time, val(3),'ro');
    end
    % plot phase time 
    phase_time = sp.getPhaseTime(vis_state,i,1);
    y1 = ylim;
    line([phase_time(1) phase_time(1)],[y1(1) y1(2)],'Color','black','LineStyle','--'); hold on;
    line([phase_time(2) phase_time(2)],[y1(1) y1(2)],'Color','black','LineStyle','--'); hold on;
    title(strcat('leg', int2str(i), ' force'));
end
end