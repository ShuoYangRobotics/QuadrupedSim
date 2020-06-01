function p_e = quad_optimal_transition_get_foot_pos_from_state(state, param)
% stance state to foot pose in earth frame
%% stance_state to variables
% COM position and pitch angle
pc = state(1:3);
roll_ang  = state(4);
pitch_ang = state(5);
yaw_ang   = state(6);

q = zeros(3,param.leg_num);
q(:,1) = state(7:9)';
q(:,2) = state(10:12)';
q(:,3) = state(13:15)';
q(:,4) = state(16:18)';

R_ec =   [ cos(yaw_ang)  -sin(yaw_ang)    0;
           sin(yaw_ang)   cos(yaw_ang)    0;
                     0               0    1]*...
         [cos(pitch_ang) 0 sin(pitch_ang);
                       0 1              0;
         -sin(pitch_ang) 0 cos(pitch_ang)]*...
         [             1              0              0;
                       0  cos(roll_ang) -sin(roll_ang);
                       0  sin(roll_ang)  cos(roll_ang)];
t_ec = [pc(1);pc(2);pc(3)];

%% get foot pos
p_e = zeros(3,param.leg_num); % foot pose in earth frame

% iterate through all legs
for i=1:param.leg_num
    T_sf = autoGen_fk_space(q(1,i), q(2,i), q(3,i));
    p_s = T_sf(1:3,4);
%     R_s = T_sf(1:3,1:3);
    p_c = param.R_cs(:,:,i)*p_s + param.t_cs(:,i);
    p_e(:,i) = R_ec*p_c + t_ec;
end

end