% Jun-1-2020
% this new model is used to study whole body dynamics control 
addpath(genpath(fullfile(pwd,'contat_lib')));
%parameters for ground plane geometry and contract
ground.stiff = 5e4;
ground.damp = 1000;
ground.height = 0.4;
ground.terrain_u = 0.7;

%% a quad param, this is a standard format
quad_param.leg_num = 4;
%%robot body size
quad_param.x_length = 0.37;
quad_param.y_length = 0.3;
quad_param.z_length = 0.15;
quad_param.chassis_density = 740;
quad_param.shoulder_size = 0.07; 
quad_param.shoulder_density = 660; 
quad_param.limb_width = 0.04;
quad_param.upper_length = 0.16;
quad_param.lower_length = 0.25;
quad_param.limb_density = 660; 
quad_param.foot_radius = 0.035; % foot density is 0
quad_param.shoulder_distance = 0.2;
quad_param.max_stretch = quad_param.upper_length + quad_param.lower_length;
quad_param.knee_damping = 0.1;
%%robot mass and inertia 
quad_param.body_mass = quad_param.chassis_density*quad_param.x_length*quad_param.y_length*quad_param.z_length;
quad_param.should_mass = quad_param.shoulder_density*quad_param.shoulder_size^3;
quad_param.upper_leg_mass = quad_param.limb_density*quad_param.limb_width*quad_param.limb_width*quad_param.upper_length;
quad_param.lower_leg_mass = quad_param.limb_density*quad_param.limb_width*quad_param.limb_width*quad_param.lower_length;

quad_param.body_inertia = [quad_param.body_mass/12*(quad_param.y_length^2+quad_param.z_length^2) 0 0;
                             0 quad_param.body_mass/12*(quad_param.x_length^2+quad_param.z_length^2) 0;
                             0 0 quad_param.body_mass/12*(quad_param.x_length^2+quad_param.y_length^2)];

quad_param.total_mass = quad_param.body_mass + 4*(quad_param.should_mass+...
    quad_param.upper_leg_mass+quad_param.lower_leg_mass);
quad_param.g = 9.805;
quad_param.leg_l1 = quad_param.upper_length;
quad_param.leg_l2 = quad_param.lower_length+quad_param.foot_radius;

planner.stand_height = quad_param.leg_l2;
body.foot_radius = 0.035; % foot density is 0
body.knee_damping = 0.1;
%% leg mounts. Used to calculate R_cs t_cs
dx = quad_param.x_length/2-quad_param.shoulder_size/2;
dy = quad_param.y_length/2+quad_param.shoulder_size/2;
quad_param.leg1_mount_x = dx;
quad_param.leg1_mount_y = dy;
quad_param.leg1_mount_angle = pi/2;
quad_param.R_cs(:,:,1) = [cos(quad_param.leg1_mount_angle) -sin(quad_param.leg1_mount_angle) 0;
               sin(quad_param.leg1_mount_angle)  cos(quad_param.leg1_mount_angle) 0;
                                         0                            0 1];
quad_param.t_cs(:,1) = [quad_param.leg1_mount_x;
               quad_param.leg1_mount_y;
                                0];

quad_param.leg2_mount_x = dx;
quad_param.leg2_mount_y = -dy;
quad_param.leg2_mount_angle = -pi/2;
quad_param.R_cs(:,:,2) = [cos(quad_param.leg2_mount_angle) -sin(quad_param.leg2_mount_angle) 0;
               sin(quad_param.leg2_mount_angle)  cos(quad_param.leg2_mount_angle) 0;
                                         0                            0 1];
quad_param.t_cs(:,2) = [quad_param.leg2_mount_x;
               quad_param.leg2_mount_y;
                                0];

quad_param.leg3_mount_x = -dx;
quad_param.leg3_mount_y = dy;
quad_param.leg3_mount_angle = pi/2;
quad_param.R_cs(:,:,3) = [cos(quad_param.leg3_mount_angle) -sin(quad_param.leg3_mount_angle) 0;
               sin(quad_param.leg3_mount_angle)  cos(quad_param.leg3_mount_angle) 0;
                                         0                            0 1];
quad_param.t_cs(:,3) = [quad_param.leg3_mount_x;
               quad_param.leg3_mount_y;
                                0];

quad_param.leg4_mount_x = -dx;
quad_param.leg4_mount_y = -dy;
quad_param.leg4_mount_angle = -pi/2;
quad_param.R_cs(:,:,4) = [cos(quad_param.leg4_mount_angle) -sin(quad_param.leg4_mount_angle) 0;
               sin(quad_param.leg4_mount_angle)  cos(quad_param.leg4_mount_angle) 0;
                                         0                            0 1];
quad_param.t_cs(:,4) = [quad_param.leg4_mount_x;
               quad_param.leg4_mount_y;
                                0];
%% joint twists
quad_param.leg_joint1_w = [0;0;1];
quad_param.leg_joint2_w = [0;1;0];
quad_param.leg_joint3_w = [0;1;0];
quad_param.leg_joint1_body_q = [-(quad_param.leg_l1+quad_param.leg_l2);0;0];
quad_param.leg_joint2_body_q = [-(quad_param.leg_l1+quad_param.leg_l2);0;0];
quad_param.leg_joint3_body_q = [-(quad_param.leg_l2);0;0];
quad_param.leg_joint1_spatial_q = [0;0;0];
quad_param.leg_joint2_spatial_q = [0;0;0];
quad_param.leg_joint3_spatial_q = [quad_param.leg_l1;0;0];
quad_param.Slist_link1 = [[quad_param.leg_joint1_w;-cross(quad_param.leg_joint1_w,quad_param.leg_joint1_spatial_q)], ...
                          [quad_param.leg_joint2_w;-cross(quad_param.leg_joint2_w,quad_param.leg_joint2_spatial_q)]];

quad_param.Slist_link2 = [[quad_param.leg_joint1_w;-cross(quad_param.leg_joint1_w,quad_param.leg_joint1_spatial_q)], ...
                          [quad_param.leg_joint2_w;-cross(quad_param.leg_joint2_w,quad_param.leg_joint2_spatial_q)], ...
                          [quad_param.leg_joint3_w;-cross(quad_param.leg_joint3_w,quad_param.leg_joint3_spatial_q)]];
quad_param.Blist_link2 = [[quad_param.leg_joint1_w;-cross(quad_param.leg_joint1_w,quad_param.leg_joint1_body_q)], ...
                          [quad_param.leg_joint2_w;-cross(quad_param.leg_joint2_w,quad_param.leg_joint2_body_q)], ...
                          [quad_param.leg_joint3_w;-cross(quad_param.leg_joint3_w,quad_param.leg_joint3_body_q)]];

                      
%% home positions, these configurations does not include offsets. They are
% just used for visualization
quad_param.M1 = [[1, 0, 0, quad_param.leg_l1/2];  % home configuration of link1
            [0, 1, 0,              0]; 
            [0, 0, 1,              0]; 
            [0, 0, 0,              1]];
quad_param.M2 = [[1, 0, 0, quad_param.leg_l1+quad_param.leg_l2/2];  % home configuration of link2
            [0, 1, 0,                           0]; 
            [0, 0, 1,                           0]; 
            [0, 0, 0,                           1]];
quad_param.M3 = [[1, 0, 0, quad_param.leg_l1+quad_param.leg_l2];  % home configuration of link2
            [0, 1, 0,                         0]; 
            [0, 0, 1,                         0]; 
            [0, 0, 0,                         1]];
        
quad_param.home_ang = [0;0;pi/2];        
% parameters for leg control 
ctrl.pos_kp = 30;
ctrl.pos_ki = 0.02;
ctrl.pos_kd = 0.01;
ctrl.vel_kp = 6.4;
ctrl.vel_ki = 0.02;
ctrl.vel_kd = 0.01;

%% optimal trajectory problem
quad_param.init_base_ang = 14/180*pi;
quad_param.init_elow_ang = 90/180*pi;

quad_param.total_time = 4;
quad_param.dt = 0.02;
quad_param.traj_pos_dim = 3;
quad_param.traj_com_ang_dim = 3;
quad_param.size_knot = 7; % there are 7 elements in one knots
quad_param.size_knot_n = 6;
quad_param.sw_knot_num = 1;  % swing pos profile knots
quad_param.st_knot_num = 25;  % stance force profile knots
quad_param.co_knot_num = 8;  % body com pos profile knots
quad_param.opt_state_size = 2*quad_param.leg_num + 2*quad_param.co_knot_num + quad_param.leg_num*quad_param.sw_knot_num +...
                 quad_param.leg_num*quad_param.st_knot_num*2 + quad_param.size_knot_n*quad_param.co_knot_num+...
                 quad_param.size_knot_n*quad_param.co_knot_num+...
                 quad_param.size_knot_n*quad_param.leg_num*quad_param.sw_knot_num+...
                 quad_param.size_knot_n*quad_param.st_knot_num*2*quad_param.leg_num;
quad_param.first_state_size = 2*quad_param.leg_num + 2*quad_param.co_knot_num + quad_param.leg_num*quad_param.sw_knot_num +...
                 quad_param.leg_num*quad_param.st_knot_num*2 + quad_param.size_knot_n*quad_param.co_knot_num+...
                 quad_param.size_knot_n*quad_param.co_knot_num+...
                 quad_param.size_knot_n*quad_param.leg_num*quad_param.sw_knot_num;
quad_param.st_knot_state_size = quad_param.size_knot_n*quad_param.st_knot_num*2*quad_param.leg_num;         


quad_param.swing_height = 0.07;
quad_param.force_scale = 100;

quad_param.terrain_u2 = ground.terrain_u;
quad_param.weight_ratio = 1.5;
quad_param.max_foot_force_vel = 55.6;
quad_param.max_foot_pos_vel   = 25.8;
quad_param.max_com_pos_vel   = 0.5;
quad_param.swing_bound = 0.08;
quad_param.com_bound = 0.264;
quad_param.com_pitch_bound = 0.4;

quad_param.num_collo_point = 32;

quad_param.opt_obj_stance_Fxy_penalty = 0.15;
quad_param.opt_obj_stance_Fz_penalty = 0.04;
quad_param.opt_obj_stance_Fvel_penalty = 0.25;
quad_param.opt_obj_com_pos_penalty = 2.0;
quad_param.opt_obj_swing_pos_penalty = 0.1;

%% parameters for LQR
quad_param.dt = 0.003;
quad_param.size_x = 24;
quad_param.size_u = 24;


