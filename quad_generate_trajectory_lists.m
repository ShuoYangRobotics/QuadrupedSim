load('quad_opt_output_5', 'opt_state_init', 'opt_state_soln','robot_state1', 'robot_state2');
addpath('../CasADi')
import casadi.*

load('casadi_state_derive_quad','casadi_sx_com_angle_func',...
                                'casadi_sx_com_pos_func',...
                                'casadi_sx_com_angle_vel_func',...
                                'casadi_sx_com_vel_func',...
                                'casadi_sx_list_force_func',...
                                'casadi_sx_list_swing_pos_func',...
                                'casadi_sx_list_swing_vel_func',...
                                'casadi_sx_list_swing_acc_func'); 
% calculate necessary start and end pos and force 
% the actual pos of p_e and com_pos need to add this reference value to
% scale back
sp = TransitionStateProcessor(quad_param);

com_pos_start = robot_state1(1:3);
com_pos_end = robot_state2(1:3);
com_ang_start = robot_state1(4:6);
com_ang_end = robot_state2(4:6);
p_e_start = quad_optimal_transition_get_foot_pos_from_state(robot_state1, quad_param);
p_e_end = quad_optimal_transition_get_foot_pos_from_state(robot_state2, quad_param);
F_e_start = quad_optimal_transition_get_foot_force_from_state(robot_state1, quad_param);
F_e_end = quad_optimal_transition_get_foot_force_from_state(robot_state2, quad_param);


t_list = 0:0.005:quad_param.total_time;  
com_pos_list = zeros(3,size(t_list,2));
com_angle_list = zeros(3,size(t_list,2));
com_vel_list = zeros(3,size(t_list,2));
com_angle_vel_list = zeros(3,size(t_list,2));
foot_pos_list = zeros(quad_param.leg_num,3,size(t_list,2)); 
foot_vel_list = zeros(quad_param.leg_num,3,size(t_list,2)); 
foot_acc_list = zeros(quad_param.leg_num,3,size(t_list,2)); 
foot_force_list = zeros(quad_param.leg_num,3,size(t_list,2)); 

% contact 1 is stance, contact 0 is swing
contact_list = zeros(quad_param.leg_num,size(t_list,2));

for i=1:size(t_list,2)
    cur_time = t_list(i);
    angle_val = casadi_sx_com_angle_func(cur_time, opt_state_soln, com_ang_start, com_ang_end);
    com_angle_list(:,i) = full(angle_val);
    angle_vel_val = casadi_sx_com_angle_vel_func(cur_time, opt_state_soln, com_ang_start, com_ang_end);
    com_angle_vel_list(:,i) = full(angle_vel_val);
    
    com_pos_val = casadi_sx_com_pos_func(cur_time, opt_state_soln, com_pos_start, com_pos_end);
    com_pos_list(:,i) = full(com_pos_val) + ref_com_pos_start;
    com_vel_val = casadi_sx_com_vel_func(cur_time, opt_state_soln, com_pos_start, com_pos_end);
    com_vel_list(:,i) = full(com_vel_val);
    
    for j = 1:quad_param.leg_num
        foot_pos_val = casadi_sx_list_swing_pos_func{j}(cur_time, opt_state_soln, p_e_start(:,j), p_e_end(:,j));
        foot_pos_list(j,:,i) = full(foot_pos_val) + ref_com_pos_start;
        
        foot_vel_val = casadi_sx_list_swing_vel_func{j}(cur_time, opt_state_soln, p_e_start(:,j), p_e_end(:,j));
        foot_vel_list(j,:,i) = full(foot_vel_val);
        
        foot_acc_val = casadi_sx_list_swing_acc_func{j}(cur_time, opt_state_soln, p_e_start(:,j), p_e_end(:,j));
        foot_acc_list(j,:,i) = full(foot_acc_val);
        
%         contact list from the phase time of the solution 
        phase_time = sp.getPhaseTime(opt_state_soln,j,1);
        if ((cur_time < phase_time(1)) || (cur_time >= phase_time(2)))
            contact_list(j,i) = 1;
        else
            contact_list(j,i) = 0;
        end
    end
    % force is more complicated
    for j = 1:quad_param.leg_num
        foot_force_val = casadi_sx_list_force_func{j}(cur_time, opt_state_soln, F_e_start(:,j), F_e_end(:,j));
        foot_force_list(j,:,i) = full(foot_force_val)*quad_param.force_scale;
    end
end

%% test LQR
% x_0 use t= 0
init_foot_pos = zeros(3,quad_param.leg_num);
for i=1:quad_param.leg_num
    init_foot_pos(:,i) = foot_pos_list(i,:,1);
    
end
x_0 = [com_pos_list(:,1); ...
             com_angle_list(:,1);...
             [0;0;0];[0;0;0];...
             reshape(init_foot_pos,3*quad_param.leg_num,1)];


% x_d_final uses t = T
init_foot_pos = zeros(3,quad_param.leg_num);
for i=1:quad_param.leg_num
    init_foot_pos(:,i) = foot_pos_list(i,:,end);
    
end
x_d_final = [com_pos_list(:,end); ...
             com_angle_list(:,end);...
             [0;0;0];[0;0;0];...
             reshape(init_foot_pos,3*quad_param.leg_num,1)];
   
% euler integration    
x = x_0;
for i=1:size(t_list,2)
    
    curr_foot_force = zeros(3,quad_param.leg_num);
    for j=1:quad_param.leg_num
        curr_foot_force(:,j) = foot_force_list(j,:,i);
    end
    curr_foot_vel = zeros(3,quad_param.leg_num);
    for j=1:quad_param.leg_num
        curr_foot_vel(:,j) = foot_vel_list(j,:,i);
    end
    curr_u = [reshape(curr_foot_force,3*quad_param.leg_num,1);...
              reshape(curr_foot_vel,3*quad_param.leg_num,1)];
 
    xdot = casadi_quad_LQR_f_func_code('quad_LQR_f_func',x,curr_u, contact_list(:,i));  
    x = x + xdot*quad_param.dt;
end

