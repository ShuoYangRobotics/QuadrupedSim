addpath('mr');
%% init necessary parameters
% init parameter
init_quad_whole_body_ctrl;

% formulate optimal stance on flat ground
stance_origin = [0; 0; quad_param.leg_l2; ...       % pos x y z, 
                 0; 0; 0; ...                       % orientation roll, pitch, yaw
                 -quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg1 angles
                 quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg2 angles
                 quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg3 angles
                 -quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg4 angles
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 1 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 2 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 3 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num];   % foot 4 force
             
stance_tgt    = [0.15; 0; quad_param.leg_l2; ...       % pos x y z, 
                    0; 0; 0; ...                       % orientation roll, pitch, yaw
                 -quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg1 angles
                 quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg2 angles
                 quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg3 angles
                 -quad_param.init_base_ang; 0; quad_param.init_elow_ang;...                % leg4 angles
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
save('quad_opt_output_6', 'opt_state_init', 'opt_state_soln','robot_state1', 'robot_state2');
%% visualize
%% visualize solution
ref_com_pos_start = stance_origin(1:3);
quad_optimal_transition_visualize(30,opt_state_init, robot_state1, robot_state2, ref_com_pos_start, quad_param);
quad_optimal_transition_visualize(31,opt_state_soln, robot_state1, robot_state2, ref_com_pos_start, quad_param);
% plot result trajectory on figure 1
quad_optimal_transition_visualize_plot3(fig_id, opt_state_soln, robot_state1, robot_state2, ref_com_pos_start, quad_param);

quad_optimal_transition_print_times(opt_state_soln, quad_param);

%% generate list
quad_generate_trajectory_lists;                                                           