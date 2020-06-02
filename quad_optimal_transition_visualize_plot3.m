function quad_optimal_transition_visualize_plot3(fig_id, vis_state, robot_state1, robot_state2, ref_com_pos_start, param)
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

t_list = 0:0.05:param.total_time;  
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
        foot_force_list{j}(:,i) = full(foot_force_val)*100;
    end
end

fig = figure(fig_id);
hold on;
com_pos_traj = com_pos_list;
plot3(com_pos_traj(1,:),com_pos_traj(2,:),com_pos_traj(3,:),'LineWidth',4, 'Color',[0.1 1 1])
for i = 1:param.leg_num
foot_pos_traj = foot_pos_list{i};
plot3(foot_pos_traj(1,:),foot_pos_traj(2,:),foot_pos_traj(3,:),'LineWidth',4, 'Color',[0 1 0])
end

end