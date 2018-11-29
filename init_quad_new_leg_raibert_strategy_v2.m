% The contact lib by Mathworks, may change to another lib written by
% Professor Hartmut Geyer
addpath(genpath(fullfile(pwd,'contat_lib')));
%parameters for ground plane geometry and contract
ground.stiff = 5e4;
ground.damp = 1000;
ground.height = 0.4;

%robot body size
body.x_length = 0.6;
body.y_length = 0.3;
body.z_length = 0.15;
body.shoulder_size = 0.07;  
body.upper_length = 0.16;
body.lower_length = 0.25;
body.foot_radius = 0.035;
body.shoulder_distance = 0.2;
body.max_stretch = body.upper_length + body.lower_length;
body.knee_damping = 0.1;

% parameters for leg control 
ctrl.pos_kp = 30;
ctrl.pos_ki = 0;
ctrl.pos_kd = 0;
ctrl.vel_kp = 4.4;
ctrl.vel_ki = 0;
ctrl.vel_kd = 0.0;

% parameters for high level plan
planner.touch_down_height = body.foot_radius; % from foot center to ground

planner.stand_s = 0;
planner.stand_u = 30*pi/180;
planner.stand_k = -60*pi/180;
stance_pos = forward_kinematics(planner.stand_s, planner.stand_u, planner.stand_k, body);
planner.stand_height = stance_pos(3);

planner.flight_height = 1.1*planner.stand_height;% when leg is flying in the air

% gait control parameters 
planner.time_circle = 3;
planner.swing_ang = 12/180*pi;
planner.init_shake_ang = 3/180*pi;

%some gait control goals
planner.tgt_body_ang = 0;
planner.tgt_body_vx = 0.3;  % tested 0.15-0.45
planner.Ts = 0.1; % for x_direction leg place
planner.Kv = 0.3;  % for x_dreiction leg place


planner.y_Ts = 0.21; % for y_direction leg place
planner.y_Kv = -0.34;  % for y_direction leg place

% state transition thredsholds
planner.state0_vel_thres = 0.05;
% the sample time of the ctrl block is set to be 0.0025 (400Hz);
planner.state0_trans_thres = 300; %(0.75 seconds)
planner.state0_swing_ang  = 10*pi/180;
planner.state0_swing_T = 1200;
planner.state12_trans_speed = 0.2;
planner.leg_swing_time = 70; %(use 0.25s to generate a motion)

%robot weight 
body_weight = 600*body.x_length*body.y_length*body.z_length;
leg_density = 660;
should_weight = leg_density*0.07^3;
upperleg_weight = leg_density*0.04*0.04*body.upper_length;
lowerleg_weight = leg_density*0.04*0.04*body.lower_length;
foot_weight = 1000*4/3*pi*body.foot_radius^3;

total_weight = body_weight + 4*(should_weight+upperleg_weight+lowerleg_weight+foot_weight);

body_inertia = diag([0.151875;0.516375;0.6075]);
