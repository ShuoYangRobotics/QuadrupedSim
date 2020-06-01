addpath('mr');
%% init necessary parameters
% init parameter
init_quad_whole_body_ctrl;

% formulate optimal stance on flat ground
stance_origin = [0; 0; quad_param.leg_l2; ...       % pos x y z, 
                 0; 0; 0; ...                       % orientation roll, pitch, yaw
                 0; 0; 90/180*pi;...                % leg1 angles
                 0; 0; 90/180*pi;...                % leg2 angles
                 0; 0; 90/180*pi;...                % leg3 angles
                 0; 0; 90/180*pi;...                % leg4 angles
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 1 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 2 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 3 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num];   % foot 4 force
             
stance_tgt    = [0; 0; quad_param.leg_l2; ...       % pos x y z, 
                 0; 0; 0; ...                       % orientation roll, pitch, yaw
                 0; 0; 90/180*pi;...                % leg1 angles
                 0; 0; 90/180*pi;...                % leg2 angles
                 0; 0; 90/180*pi;...                % leg3 angles
                 0; 0; 90/180*pi;...                % leg4 angles
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 1 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 2 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num;... % foot 3 force
                 0;0;quad_param.total_mass*quad_param.g/quad_param.leg_num];   % foot 4 force             