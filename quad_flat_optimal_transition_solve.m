function [opt_state_init, opt_state_soln, lambda, robot_state1, robot_state2] = ...
    quad_flat_optimal_transition_solve(robot_state1, robot_state2, param, terrain)

addpath('../CasADi')
import casadi.*   
lambda = [];

%% calculate an initial transition state
transition_state_init = quad_optimal_transition_init_state(param, robot_state1, robot_state2);
save('quad_optimal_transition_init_state', 'transition_state_init', 'robot_state1', 'robot_state2');


%% construct fmincon
% load('optimal_transition_state_soln', 'transition_state_soln')
% problem.x0 = transition_state_soln;

problem.x0 = transition_state_init;
problem.lb = []; problem.ub = [];  % put bound constraints in Aineq bineq
problem.Aeq = [];problem.beq = [];

% Aineq bineq also includes variable bounds
[problem.Aineq, problem.bineq] = quad_optimal_transition_linear_ineq(transition_state_init,robot_state1, robot_state2, param);

% load casadi functions, "derive_optimal_transition_casadi_funcs" must be
% called first
load('casadi_derive_quad','casadi_quad_sx_obj_func','casadi_quad_sx_obj_grad_func', ...
     'casadi_quad_sx_ceq_block_func', 'casadi_quad_sx_ceq_block_grad_func');

% 4. objective
problem.objective = @(state) (quad_optimal_transition_objective(state,robot_state1, robot_state2, param, casadi_quad_sx_obj_func, casadi_quad_sx_obj_grad_func));

% 5. constraint 
problem.nonlcon = @(state) (quad_optimal_transition_constraints(state,robot_state1, robot_state2, param, casadi_quad_sx_ceq_block_func, casadi_quad_sx_ceq_block_grad_func));

% 6. options 
problem.solver = 'fmincon';
problem.options = optimoptions('fmincon','Display','iter',...
    'Algorithm','interior-point','MaxFunctionEvaluations', 10000, 'MaxIterations', 200, ...
        'StepTolerance', 1e-8, ...
        'ConstraintTolerance', 1e-7,...
    'SpecifyObjectiveGradient',true, ...
    'SpecifyConstraintGradient',true);

%% solve the problem
tic;
[transition_state_soln, fval,exitflag,output,lambda,grad,hessian]= fmincon(problem);
save('optimal_transition_state_soln', 'transition_state_soln')
nlpTime = toc 


opt_state_init = [transition_state_init];
opt_state_soln = [transition_state_soln];
end