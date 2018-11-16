addpath(genpath(fullfile(pwd,'contat_lib')));
%parameters for ground contract
ground.stiff = 5e3;
ground.damp = 1000;
ground.height = 0.4;

%body size
body.x_length = 0.6;
body.y_length = 0.3;
body.z_length = 0.15;
body.shoulder_size = 0.07;  
body.upper_length = 0.16;
body.lower_length = 0.25;
body.foot_radius = 0.035;
body.shoulder_distance = 0.2;

% parameters for leg control 
ctrl.pos_kp = 30;
ctrl.pos_ki = 0;
ctrl.pos_kd = 0;
ctrl.vel_kp = 4.4;
ctrl.vel_ki = 0;
ctrl.vel_kd = 0;

% parameters for high level plan
planner.touch_down_height = body.foot_radius; % from foot center to ground

planner.stand_s = 0;
planner.stand_u = 35*pi/180;
planner.stand_k = -60*pi/180;
stance_pos = forward_kinematics(planner.stand_s, planner.stand_u, planner.stand_k, body);
planner.stand_height = stance_pos(3);

planner.flight_height = 0.6*planner.stand_height;% when leg is flying in the air

% gait control parameters 
planner.time_circle = 0.5;


%robot weight 
body_weight = 600*body.x_length*body.y_length*body.z_length;
should_weight = 660*0.07^3;
upperleg_weight = 660*0.04*0.04*body.upper_length;
lowerleg_weight = 660*0.04*0.04*body.lower_length;
foot_weight = 1000*4/3*pi*body.foot_radius^3;

total_weight = body_weight + 4*(should_weight+upperleg_weight+lowerleg_weight+foot_weight)
