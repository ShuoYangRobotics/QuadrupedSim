function out_p = spline_from_knots_casadi(cur_t, knots)
% added 0505
% I found that casadi support logical operations so
% there is no need to use logistic function!
%% add casadi
addpath('../CasADi')
import casadi.*
total_knot_num = size(knots,2);
traj_pos_dim = (size(knots,1)-1)/2;
%% find time indice
% knots_idx = 1;
% 
% while (t>=time_list(knots_idx))
%     knots_idx = knots_idx + 1;
%     if (knots_idx == size(time_idx_list,2))
%         break;
%     end
% end
% 
% knot1_idx = knots_idx-1;
% knot2_idx = knots_idx;
condition_list = SX.zeros(total_knot_num-1,1);
for i=1:total_knot_num-2
    condition_list(i) = if_else(cur_t >= knots(1,i) & cur_t < knots(1,i+1), ...
         1, 0);

end
condition_list(total_knot_num-1) = if_else(cur_t >= knots(1,total_knot_num-1) & cur_t <= knots(1,total_knot_num-1+1), ...
         1, 0);
% idx = find(condition_list);
%% construct spline
out_p = SX.zeros(traj_pos_dim,1);
for idx=1:total_knot_num-1
    knot1 = knots(:,idx);
    knot2 = knots(:,idx+1);
    t0 = knot1(1);
    t1 = knot2(1);
    t = (cur_t-t0)/(t1-t0);
    x0 = knot1(2:2+traj_pos_dim-1);
    dx0 = knot1(2+traj_pos_dim:2+2*traj_pos_dim-1);
    x1 = knot2(2:2+traj_pos_dim-1);
    dx1 = knot2(2+traj_pos_dim:2+2*traj_pos_dim-1);
    p =  x0*(2*t^3-3*t^2 + 1) + ...
        dx0*(  t^3-2*t^2 + t)*(t1-t0) + ...
         x1*(-2*t^3+3*t^2)    + ...
        dx1*(t^3-t^2)*(t1-t0); 
    out_p = out_p+p*condition_list(idx);     
end

end