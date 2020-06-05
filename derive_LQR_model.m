% this code derives a single model

%% add modern robotics
addpath('mr')
%% add casadi
addpath('../CasADi')
import casadi.*
init_quad_whole_body_ctrl;
param = quad_param;

% com position in world frame
com_pos_e = SX.sym('p_com_e',3,1);   
% com orientation in world frame
com_ang_e = SX.sym('ang_com_e',3,1); % roll pitch yaw
% com velocity in world frame 
com_vel_e = SX.sym('v_com_e',3,1);
% com angular velocity in COM frame
com_w_c = SX.sym('w_com_c',3,1);
% foot position % dim 12 
foot_pos = SX.sym('foot_pos',3*param.leg_num, 1);   %3*4
foot_vel = SX.sym('foot_vel',3*param.leg_num, 1);
% contact state
contact = SX.sym('contact',param.leg_num, 1);

% Foot force in world frame % dim 12 
F_e = SX.sym('F_e',3*param.leg_num, 1);

% state dim 24
x = [com_pos_e; com_ang_e; com_vel_e; com_w_c; foot_pos];    
% control dim 24
u = [F_e;foot_vel];



% helper values
%% orientation
roll_ang  = com_ang_e(1);
pitch_ang = com_ang_e(2);
yaw_ang   = com_ang_e(3);
R_ec =   [ cos(yaw_ang)  -sin(yaw_ang)    0;
           sin(yaw_ang)   cos(yaw_ang)    0;
                     0               0    1]*...
         [cos(pitch_ang) 0 sin(pitch_ang);
                       0 1              0;
         -sin(pitch_ang) 0 cos(pitch_ang)]*...
         [         1              0              0;
                   0  cos(roll_ang) -sin(roll_ang);
                   0  sin(roll_ang)  cos(roll_ang)];
% convert euler angluar velocity to body velocity
% w_b = B*com_ang_vel;     
% com_ang_vel = Binv*w_b;               
B = [1  0 -sin(pitch_ang);
     0  cos(roll_ang) sin(roll_ang)*cos(pitch_ang);
     0 -sin(roll_ang) cos(roll_ang)*cos(pitch_ang)];      
Binv = [1  sin(roll_ang)*tan(pitch_ang) cos(roll_ang)*tan(pitch_ang);
        0  cos(roll_ang)                               -sin(roll_ang);
        0  sin(roll_ang)/cos(pitch_ang) cos(roll_ang)/cos(pitch_ang)];            
    
% xdot = [com_pos_e_dot; com_ang_e_dot; com_vel_e_dot; com_w_c_dot; foot_pos_dot];    
com_pos_e_dot = com_vel_e;
com_ang_e_dot = Binv*com_w_c;
com_vel_e_dot = [0;0;-param.g] + 1/param.total_mass*reshape(F_e,3,param.leg_num)*reshape(contact,param.leg_num,1);
I = param.body_inertia;
w_acc = -VecToso3(com_w_c)*I*com_w_c;
for i = 1:param.leg_num
    % no dynamics yet
    swing_dynamics = VecToso3(param.t_cs(:,i))*R_ec'*[0;0;(param.upper_leg_mass+param.lower_leg_mass)*-param.g];
    p_c = R_ec'*(foot_pos((i-1)*3+1:(i-1)*3+3) - com_pos_e);
    w_acc = w_acc + swing_dynamics*(1-contact(i)) + contact(i)*VecToso3(p_c)*R_ec'*F_e((i-1)*3+1:(i-1)*3+3);
end
com_w_c_dot = inv(I)*(w_acc);     %  I^-1(M - w x Iw) = wdot 
foot_pos_dot = foot_vel;

xdot = [com_pos_e_dot; com_ang_e_dot; com_vel_e_dot; com_w_c_dot; foot_pos_dot];

A = xdot.jacobian(x);
B = xdot.jacobian(u);

casadi_quad_LQR_f_func = Function('quad_LQR_f_func', {x,u, contact},{xdot});
casadi_quad_LQR_A_func = Function('quad_LQR_A_func', {x,u, contact},{A});
casadi_quad_LQR_B_func = Function('quad_LQR_B_func', {x,u, contact},{B});

disp('generate model code')   
opts = struct('mex', true);
casadi_quad_LQR_f_func.generate('casadi_quad_LQR_f_func_code.c', opts);              
casadi_quad_LQR_A_func.generate('casadi_quad_LQR_A_func_code.c', opts);             
casadi_quad_LQR_B_func.generate('casadi_quad_LQR_B_func_code.c', opts); 

% mex casadi_quad_LQR_f_func_code.c
% mex casadi_quad_LQR_A_func_code.c
% mex casadi_quad_LQR_B_func_code.c

mex -v GCC='/usr/local/gcc-6.3/bin/gcc-6.3' casadi_quad_LQR_f_func_code.c
mex -v GCC='/usr/local/gcc-6.3/bin/gcc-6.3' casadi_quad_LQR_A_func_code.c
mex -v GCC='/usr/local/gcc-6.3/bin/gcc-6.3' casadi_quad_LQR_B_func_code.c