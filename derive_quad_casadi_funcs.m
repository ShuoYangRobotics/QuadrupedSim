%% add modern robotics
addpath('mr')
%% add casadi
addpath('../CasADi')
import casadi.*
init_quad_whole_body_ctrl;
param = quad_param;
cur_t = SX.sym('cur_t',1);
n = SX.sym('n',1);
dt = SX.sym('dt',1);  %cur_t is the current time. 
sym_state = SX.sym('opt_state',param.opt_state_size,1);
sym_p_0 = SX.sym('p0',3,param.leg_num);
sym_p_T = SX.sym('pT',3,param.leg_num);
sym_com_0 = SX.sym('com0',3,1);
sym_com_T = SX.sym('comT',3,1);
sym_com_ang_0 = SX.sym('com_ang0',3,1);
sym_com_ang_T = SX.sym('com_angT',3,1);
sym_F_0 = SX.sym('F0',3,param.leg_num);
sym_F_T = SX.sym('FT',3,param.leg_num);


% given current time cur_t, intepolate state com pos com pitch foot
% pos foot force
% state processor
sp = TransitionStateProcessor(param);

%% get force
addpath('../CasADi')
import casadi.*
casadi_sx_list_force = {};    % force = H*state_knot+b

for i = 1:param.leg_num
    force_knot_time = SX.zeros(1,2+2+2*param.st_knot_num);
    force_knot_value = SX.zeros(param.traj_pos_dim*2,2+2+2*param.st_knot_num);
    % boundary knots    
    default_F_0 = sym_F_0(:,i);
    default_F_T = sym_F_T(:,i);
    force_knot_time(1) = 0;
    force_knot_value(:,1) = [default_F_0;[0;0;0]];
    force_knot_time(2+2+2*param.st_knot_num) = param.total_time;
    force_knot_value(:,2+2+2*param.st_knot_num) = [default_F_T;[0;0;0]];
    % phase knots
    phase_time = sp.getPhaseTime(sym_state, i, 1);
    force_knot_time(1+param.st_knot_num+1) = phase_time(1);
    force_knot_value(:,1+param.st_knot_num+1) = [0;0;0;0;0;0];
    force_knot_time(1+param.st_knot_num+2) = phase_time(2);
    force_knot_value(:,1+param.st_knot_num+2) = [0;0;0;0;0;0];
    % other knots
    for j = 1:param.st_knot_num
        leg_stance_time = sp.getLegStanceTime(sym_state, i, j, 1);
        leg_stance_knot = sp.getLegStanceKnot(sym_state, i, j, 1);
        force_knot_time(1+j) = leg_stance_time;
        force_knot_value(:,1+j) = leg_stance_knot;
    end
    for j = 1:param.st_knot_num
        leg_stance_time = sp.getLegStanceTime(sym_state, i, param.st_knot_num+j, 1);
        leg_stance_knot = sp.getLegStanceKnot(sym_state, i, param.st_knot_num+j, 1);        
        force_knot_time(1+param.st_knot_num+2+j) = leg_stance_time;
        force_knot_value(:,1+param.st_knot_num+2+j) = leg_stance_knot;
    end
    
    % get force
    knots = [force_knot_time;force_knot_value];
    force = spline_from_knots_casadi(cur_t, knots);
    
    % store casadi expression
    func_name = strcat('force', int2str(i), '_func');
    func = Function(func_name,{cur_t, sym_state, default_F_0, default_F_T},...
      {force},...
      {'cur_t','state', 'F0', 'FT'},{'F'});
    casadi_sx_list_force_func{i} = func;
end

%% get COM pos
addpath('../CasADi')
import casadi.*
com_pos_knot_time = SX.zeros(1,2+param.co_knot_num);
com_pos_knot_value = SX.zeros(param.traj_pos_dim*2,2+param.co_knot_num);
% boundary knots
com_pos_knot_time(1) = 0;
com_pos_knot_value(:,1) = [sym_com_0;zeros(param.traj_pos_dim,1)];
com_pos_knot_time(2+param.co_knot_num) = param.total_time;
com_pos_knot_value(:,2+param.co_knot_num) = [sym_com_T;zeros(param.traj_pos_dim,1)];
for i=1:param.co_knot_num
com_pos_knot_time(1+i) = sp.getComPosTime(sym_state,i, 1);
com_pos_knot_value(:,1+i) = sp.getComPosKnot(sym_state,i, 1);
end

com_knots = [com_pos_knot_time;com_pos_knot_value];
com_val = spline_from_knots_casadi(cur_t, com_knots);
casadi_sx_com_pos_func = Function('com_pos',{cur_t, sym_state, sym_com_0, sym_com_T},...
      {com_val},...
      {'cur_t','state', 'com0', 'comT'},{'com_pos'});
  
%% get COM angle
addpath('../CasADi')
import casadi.*
com_angle_knot_time = SX.zeros(1,2+param.co_knot_num);
com_angle_knot_value = SX.zeros(param.traj_pos_dim*2,2+param.co_knot_num);
% boundary knots
com_angle_knot_time(1) = 0;
com_angle_knot_value(:,1) = [sym_com_ang_0;zeros(param.traj_pos_dim,1)];
com_angle_knot_time(2+param.co_knot_num) = param.total_time;
com_angle_knot_value(:,2+param.co_knot_num) = [sym_com_ang_T;zeros(param.traj_pos_dim,1)];
for i=1:param.co_knot_num
com_angle_knot_time(1+i) = sp.getComAngTime(sym_state,i, 1);
com_angle_knot_value(:,1+i) = sp.getComAngKnot(sym_state,i, 1);
end
com_angle_knots = [com_angle_knot_time;com_angle_knot_value];
com_angle_val = spline_from_knots_casadi(cur_t, com_angle_knots);

casadi_sx_com_angle_func = Function('com_angle',{cur_t, sym_state, sym_com_ang_0, sym_com_ang_T},...
      {com_angle_val},...
      {'cur_t','state', 'com_angle0', 'com_angleT'},{'com_ang'});
  
%% get leg pos
addpath('../CasADi')
import casadi.*
casadi_sx_list_swing_pos = {};
for i = 1:param.leg_num
    swing_knot_time = SX.zeros(1,2+2+param.sw_knot_num);
    swing_knot_value = SX.zeros(param.traj_pos_dim*2,2+2+param.sw_knot_num);
    default_pos_0 = sym_p_0(:,i);
    default_pos_T = sym_p_T(:,i);    
    % boundary knots    
    swing_knot_time(1) = 0;
    swing_knot_value(:,1) = [default_pos_0;zeros(param.traj_pos_dim,1)];
    swing_knot_time(2+2+param.sw_knot_num) = param.total_time;
    swing_knot_value(:,2+2+param.sw_knot_num) = [default_pos_T;zeros(param.traj_pos_dim,1)];
    % phase knots
    phase_time = sp.getPhaseTime(sym_state, i, 1);
    swing_knot_time(2) = phase_time(1);
    swing_knot_value(:,2) = [default_pos_0;zeros(param.traj_pos_dim,1)];
    swing_knot_time(1+2+param.sw_knot_num) = phase_time(2);
    swing_knot_value(:,1+2+param.sw_knot_num) = [default_pos_T;zeros(param.traj_pos_dim,1)];
    % other knots
    for j = 1:param.sw_knot_num
        leg_swing_time = sp.getLegSwingTime(sym_state, i, j, 1);
        leg_swing_knot = sp.getLegSwingKnot(sym_state, i, j, 1);        
        swing_knot_time(2+j) = leg_swing_time;
        swing_knot_value(:,2+j) = leg_swing_knot;
    end
    % get swing pos
    knots = [swing_knot_time;swing_knot_value];
    swing_pos = spline_from_knots_casadi(cur_t, knots);
    
    % store casadi expression
    func_name = strcat('swing', int2str(i), '_func');
    func = Function(func_name,{cur_t, sym_state, default_pos_0, default_pos_T},...
      {swing_pos},...
      {'cur_t','state', 'pos_0', 'pos_T'},{'F'});
    casadi_sx_list_swing_pos_func{i} = func;
end  

%% derive obj 
addpath('../CasADi')
import casadi.*
obj = SX.zeros(1,1);
% first minimize force YF*Q*YF
obj_F = SX.zeros(1,1);
force_Q = diag([param.opt_obj_stance_Fxy_penalty;
                param.opt_obj_stance_Fxy_penalty;
                param.opt_obj_stance_Fz_penalty;
                param.opt_obj_stance_Fvel_penalty; 
                param.opt_obj_stance_Fvel_penalty;
                param.opt_obj_stance_Fvel_penalty]);
for i = 1:param.leg_num
    % get force knot value 
    state_knot = SX.zeros(2*param.st_knot_num*param.traj_pos_dim*2,1);
    for j = 1:param.st_knot_num*2
        leg_stance_knot = sp.getLegStanceKnot(sym_state, i, j, 1);
        state_knot((j-1)*param.traj_pos_dim*2+1:(j-1)*param.traj_pos_dim*2+6) = leg_stance_knot;    
        obj_F = obj_F + leg_stance_knot'*force_Q*leg_stance_knot;
    end
end
obj = obj + obj_F;
% then swing pos
swing_pos_Q = diag([param.opt_obj_swing_pos_penalty;
                    param.opt_obj_swing_pos_penalty;
                    param.opt_obj_swing_pos_penalty]);
for i=1:param.leg_num
    average_swing_pos = (sym_p_0(:,i) + sym_p_T(:,i))*0.5 + [0;0;param.swing_height];
    for j = 1:param.sw_knot_num
    	leg_swing_knot = sp.getLegSwingKnot(sym_state, i, j, 1);
        diff_swing_pos = leg_swing_knot(1:param.traj_pos_dim) - average_swing_pos;
        obj = obj + diff_swing_pos'*swing_pos_Q*diff_swing_pos;
    end
end

casadi_quad_sx_obj_func = Function('obj_function',{sym_state, sym_p_0, sym_p_T, sym_com_0, sym_com_T, sym_com_ang_0, sym_com_ang_T},...
      {obj},...
      {'state', 'pos_start', 'pos_end','com_start','com_end', 'com_ang_start', 'com_ang_end'},{'obj'});
obj_grad = obj.jacobian(sym_state);
casadi_quad_sx_obj_grad_func = Function('obj_grad_function',{sym_state, sym_p_0, sym_p_T, sym_com_0, sym_com_T, sym_com_ang_0, sym_com_ang_T},...
      {obj_grad},...
      {'state', 'pos_start', 'pos_end','com_start','com_end', 'com_ang_start', 'com_ang_end'},{'obj_grad'});


%% derive constraint (equality) 
addpath('../CasADi')
import casadi.*
% ceq = SX.zeros(6*param.num_collo_point,1);
% t_list = SX.sym('ct', param.num_collo_point, 1);
% for pt_idx = 1:param.num_collo_point
%     com_pos = casadi_sx_com_pos_func(t_list(pt_idx), sym_state, sym_com_0, sym_com_T);
%     com_ang = casadi_sx_com_angle_func(t_list(pt_idx), sym_state, sym_com_ang_0, sym_com_ang_T);
%     
%     roll_ang = com_ang(1);
%     pitch_ang = com_ang(2);
%     yaw_ang = com_ang(3);
%     R_ec =   [ cos(yaw_ang)  -sin(yaw_ang)    0;
%            sin(yaw_ang)   cos(yaw_ang)    0;
%                      0               0    1]*...
%          [cos(pitch_ang) 0 sin(pitch_ang);
%                        0 1              0;
%          -sin(pitch_ang) 0 cos(pitch_ang)]*...
%          [             1              0              0;
%                        0  cos(roll_ang) -sin(roll_ang);
%                        0  sin(roll_ang)  cos(roll_ang)];   
%     grasp_mtx = SX.zeros(6,3*param.leg_num);
%     foot_pos = SX.zeros(param.traj_pos_dim, param.leg_num);
%     foot_force = SX.zeros(param.traj_pos_dim*param.leg_num,1);
%     for i=1:param.leg_num
%         foot_pos(:,i)   = casadi_sx_list_swing_pos_func{i}(t_list(pt_idx),sym_state,sym_p_0(:,i),sym_p_T(:,i));
%         foot_force(1+3*(i-1):3+3*(i-1)) = casadi_sx_list_force_func{i}(t_list(pt_idx), sym_state, sym_F_0(:,i), sym_F_T(:,i));
%         grasp_mtx(1:3,1+3*(i-1):3+3*(i-1)) = eye(3);
%         grasp_mtx(4:6,1+3*(i-1):3+3*(i-1)) = VecToso3(R_ec'*(foot_pos(:,i)-com_pos))*R_ec';
%         
%     end
%     com_vel = com_pos.jacobian(t_list(pt_idx));
%     com_acc = com_vel.jacobian(t_list(pt_idx));    
%     com_ang_vel = com_ang.jacobian(t_list(pt_idx));
%     com_ang_acc = com_ang.jacobian(t_list(pt_idx));
%     
%     % convert euler angluar velocity to body velocity
%     % w_b = B*com_ang_vel;            
%     B = [1  0 -sin(pitch_ang);
%          0  cos(roll_ang) sin(roll_ang)*cos(pitch_ang);
%          0 -sin(roll_ang) cos(roll_ang)*cos(pitch_ang)];
%      
%     wb = B*com_ang_vel;
%     dwb = B*com_ang_acc;
%     
%     % body inertia
%     I = [param.total_mass/100/12*(0.4^2+0.3^2) 0 0;
%          0 param.total_mass/100/12*(0.5^2+0.3^2) 0;
%          0 0 param.total_mass/100/12*(0.5^2+0.4^2)];
%     grasp_tgt = [[0;
%                   0;
%                   param.total_mass/100*param.g]+param.total_mass/100*com_acc;
%                  I*dwb+VecToso3(wb)*I*wb]; 
%     ceq(1+6*(pt_idx-1):6*pt_idx) = grasp_mtx*foot_force - grasp_tgt;  
% end
% casadi_sx_ceq_func = Function('ceq_function',{t_list, sym_state, sym_p_0, sym_p_T, sym_F_0, sym_F_T, sym_com_0, sym_com_T, sym_com_ang_0, sym_com_ang_T},...
%       {ceq},...
%       {'t_list','state', 'pos_start', 'pos_end', 'F_start','F_end','com_start','com_end', 'com_ang_start', 'com_ang_end'},{'ceq'});
% ceq_grad = ceq.jacobian(sym_state);
% casadi_sx_ceq_grad_func = Function('ceq_grad_function',{t_list, sym_state, sym_p_0, sym_p_T, sym_F_0, sym_F_T, sym_com_0, sym_com_T, sym_com_ang_0, sym_com_ang_T},...
%       {ceq_grad},...
%       {'t_list','state', 'pos_start', 'pos_end', 'F_start','F_end','com_start','com_end', 'com_ang_start', 'com_ang_end'},{'ceq_grad'});

%% a block of 
com_pos = casadi_sx_com_pos_func(cur_t, sym_state, sym_com_0, sym_com_T);
com_ang = casadi_sx_com_angle_func(cur_t, sym_state, sym_com_ang_0, sym_com_ang_T);

roll_ang = com_ang(1);
pitch_ang = com_ang(2);
yaw_ang = com_ang(3);
R_ec =   [ cos(yaw_ang)  -sin(yaw_ang)    0;
       sin(yaw_ang)   cos(yaw_ang)    0;
                 0               0    1]*...
     [cos(pitch_ang) 0 sin(pitch_ang);
                   0 1              0;
     -sin(pitch_ang) 0 cos(pitch_ang)]*...
     [             1              0              0;
                   0  cos(roll_ang) -sin(roll_ang);
                   0  sin(roll_ang)  cos(roll_ang)];   
grasp_mtx = SX.zeros(6,3*param.leg_num);
foot_pos = SX.zeros(param.traj_pos_dim, param.leg_num);
foot_force = SX.zeros(param.traj_pos_dim*param.leg_num,1);
for i=1:param.leg_num
    foot_pos(:,i)   = casadi_sx_list_swing_pos_func{i}(cur_t,sym_state,sym_p_0(:,i),sym_p_T(:,i));
    foot_force(1+3*(i-1):3+3*(i-1)) = casadi_sx_list_force_func{i}(cur_t, sym_state, sym_F_0(:,i), sym_F_T(:,i));
    grasp_mtx(1:3,1+3*(i-1):3+3*(i-1)) = eye(3);
    grasp_mtx(4:6,1+3*(i-1):3+3*(i-1)) = VecToso3(R_ec'*(foot_pos(:,i)-com_pos))*R_ec';

end
com_vel = com_pos.jacobian(cur_t);
com_acc = com_vel.jacobian(cur_t);    
com_ang_vel = com_ang.jacobian(cur_t);
com_ang_acc = com_ang_vel.jacobian(cur_t);

% convert euler angluar velocity to body velocity
% w_b = B*com_ang_vel;            
B = [1  0 -sin(pitch_ang);
     0  cos(roll_ang) sin(roll_ang)*cos(pitch_ang);
     0 -sin(roll_ang) cos(roll_ang)*cos(pitch_ang)];

wb = B*com_ang_vel;
dwb = B*com_ang_acc;

% body inertia
I = quad_param.body_inertia/100;
grasp_tgt = [R_ec'*[0;
              0;
              param.total_mass/100*param.g]+param.total_mass/100*R_ec'*com_acc;
             I*dwb+VecToso3(wb)*I*wb]; 

% swing foot dynamics
for i=1:param.leg_num
    phase_time = sp.getPhaseTime(sym_state,i,1);
    add_weight  = if_else(cur_t >= phase_time(1) & cur_t < phase_time(2), 1, 0);
    grasp_tgt = grasp_tgt + add_weight*[[0;0;0];...
       -VecToso3(param.t_cs(:,i))*R_ec'*[0;0;(param.upper_leg_mass+param.lower_leg_mass)/100*param.g]];
%     % no dynamics yet
%     foot_pos = foot_pos(:,i);
%     foot_vel = foot_pos.jacobian(cur_t);
%     foot_acc = foot_vel.jacobian(cur_t); 
%     grasp_tgt = grasp_tgt + [(param.upper_leg_mass+param.lower_leg_mass)*R_ec'*foot_acc;]
end

         
ceq_block = grasp_mtx*foot_force - grasp_tgt;  
casadi_quad_sx_ceq_block_func = Function('ceq_block_function',{cur_t, sym_state, sym_p_0, sym_p_T, sym_F_0, sym_F_T, sym_com_0, sym_com_T, sym_com_ang_0, sym_com_ang_T},...
      {ceq_block},...
      {'t','state', 'pos_start', 'pos_end', 'F_start','F_end','com_start','com_end', 'com_ang_start', 'com_ang_end'},{'ceq_block'});
ceq_block_grad = ceq_block.jacobian(sym_state);
casadi_quad_sx_ceq_block_grad_func = Function('ceq_block_grad_function',{cur_t, sym_state, sym_p_0, sym_p_T, sym_F_0, sym_F_T, sym_com_0, sym_com_T, sym_com_ang_0, sym_com_ang_T},...
      {ceq_block_grad},...
      {'t','state', 'pos_start', 'pos_end', 'F_start','F_end','com_start','com_end', 'com_ang_start', 'com_ang_end'},{'ceq_block_grad'});
  
%% save and generate code
addpath('../CasADi')
import casadi.*
disp('save code')
save('casadi_derive_quad','casadi_quad_sx_obj_func',...
                          'casadi_quad_sx_obj_grad_func',...
                          'casadi_quad_sx_ceq_block_func',...
                          'casadi_quad_sx_ceq_block_grad_func'); 
save('casadi_state_derive_quad','casadi_sx_com_angle_func',...
                               'casadi_sx_com_pos_func',...
                               'casadi_sx_list_force_func',...
                               'casadi_sx_list_swing_pos_func'); 
                           
disp('generate state code')   
opts = struct('mex', true,'cpp',true);

casadi_sx_com_angle_func.generate('casadi_sx_com_angle_func_code.cpp');              
casadi_sx_com_pos_func.generate('casadi_sx_com_pos_func_code.cpp');  

C_force = CodeGenerator('casadi_sx_list_force_func_code.cpp');
for i=1:param.leg_num
C_force.add(casadi_sx_list_force_func{i});
end
C_force.generate();

C_pos = CodeGenerator('casadi_sx_list_swing_pos_func_code.cpp');
for i=1:param.leg_num
    C_pos.add(casadi_sx_list_swing_pos_func{i});
end
C_pos.generate();                           
                           