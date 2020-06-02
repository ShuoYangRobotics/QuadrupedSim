addpath('mr');
%% init necessary parameters
% init parameter
init_quad_whole_body_ctrl;

% formulate optimal stance on flat ground
stance_origin = [0; 0; quad_param.leg_l2; ...       % pos x y z, 
                 0; 0; 0; ...                       % orientation roll, pitch, yaw
                 -10/180*pi; 0; 90/180*pi;...                % leg1 angles
                 10/180*pi; 0; 90/180*pi;...                % leg2 angles
                 10/180*pi; 0; 90/180*pi;...                % leg3 angles
                 -10/180*pi; 0; 90/180*pi;...                % leg4 angles
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 1 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 2 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 3 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num];   % foot 4 force
             
stance_tgt    = [0.15; 0; quad_param.leg_l2; ...       % pos x y z, 
                    0; 0; 0; ...                       % orientation roll, pitch, yaw
                 -10/180*pi; 0; 90/180*pi;...                % leg1 angles
                 10/180*pi; 0; 90/180*pi;...                % leg2 angles
                 10/180*pi; 0; 90/180*pi;...                % leg3 angles
                 -10/180*pi; 0; 90/180*pi;...                % leg4 angles
                 0; 0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 1 force
                    0; 0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 2 force
                    0; 0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 3 force
                    0; 0;quad_param.total_mass*quad_param.g/quad_param.leg_num];   % foot 4 force 
                                
%% plot two stance poses
fig_id = 1;
fig = figure(fig_id);
set(gcf, 'units','normalized','outerposition',[0 0.2 0.3 0.7]);

clf; hold on;
% show two robot stances
draw_quad_robot_stance(fig, stance_origin, quad_param, 1)
draw_quad_robot_stance(fig, stance_tgt, quad_param, 0.2)       

% adjust view point of the figure 
com_pos = stance_origin(1:3);
set(gca,'CameraPosition',[com_pos(1)+4 com_pos(2)+6 com_pos(3)+6]);
set(gca,'CameraTarget',[com_pos(1) com_pos(2) com_pos(3)]);
set(gca,'CameraUpVector',[0 0 1]);
set(gca,'CameraViewAngle',6.6765);
set(gca, 'XLim', [com_pos(1)-1.2 com_pos(1)+1.6])
set(gca, 'YLim', [com_pos(2)-1.2 com_pos(2)+1.2])
set(gca, 'ZLim', [com_pos(3)-1.1 com_pos(3)+0.6])
drawnow;

%% optimal
[opt_state_init, opt_state_soln, lambda, robot_state1, robot_state2] = ...
    quad_flat_optimal_transition_solve(stance_origin, stance_tgt, quad_param);
%%
save('quad_opt_output_3', 'opt_state_init', 'opt_state_soln','robot_state1', 'robot_state2');
%% visualize
%% visualize solution
ref_com_pos_start = stance_origin(1:3);
quad_optimal_transition_visualize(30,opt_state_init, robot_state1, robot_state2, ref_com_pos_start, quad_param);
quad_optimal_transition_visualize(31,opt_state_soln, robot_state1, robot_state2, ref_com_pos_start, quad_param);
% plot result trajectory on figure 1
quad_optimal_transition_visualize_plot3(fig_id, opt_state_soln, robot_state1, robot_state2, ref_com_pos_start, quad_param);


%%
load('quad_opt_output_3', 'opt_state_init', 'opt_state_soln','robot_state1', 'robot_state2');
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
foot_pos_list = zeros(param.leg_num,3,size(t_list,2)); 
foot_force_list = zeros(param.leg_num,3,size(t_list,2)); 

for i=1:size(t_list,2)
    cur_time = t_list(i);
    angle_val = casadi_sx_com_angle_func(cur_time, opt_state_soln, com_ang_start, com_ang_end);
    com_angle_list(:,i) = full(angle_val);
    
    com_pos_val = casadi_sx_com_pos_func(cur_time, opt_state_soln, com_pos_start, com_pos_end);
    com_pos_list(:,i) = full(com_pos_val) + ref_com_pos_start;
    
    for j = 1:param.leg_num
        foot_pos_val = casadi_sx_list_swing_pos_func{j}(cur_time, opt_state_soln, p_e_start(:,j), p_e_end(:,j));
        foot_pos_list(j,:,i) = full(foot_pos_val) + ref_com_pos_start;
    end
    % force is more complicated
    for j = 1:param.leg_num
        foot_force_val = casadi_sx_list_force_func{j}(cur_time, opt_state_soln, F_e_start(:,j), F_e_end(:,j));
        foot_force_list(j,:,i) = full(foot_force_val)*100;
    end
end