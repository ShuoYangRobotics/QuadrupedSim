init_quad_new_leg_raibert_strategy_return_map;
in_speed_list = 0.05:0.03:0.55;
out_speed_list = zeros(size(in_speed_list));
for i = 1:size(in_speed_list,2)
    planner.init_speed = in_speed_list(i);
    sim('quad_new_leg_raibert_strategy_return_map');
    planner.init_speed
    out_speed_list(i) = body_vel(end,1)
end

plot(in_speed_list,out_speed_list);
axis equal
hold on;
plot(0:0.001:0.6,0:0.001:0.6);