function x = opt_ik_problem(x0,p_s)
  fmincon_target = @(x)(target(x, p_s));
  x = fmincon(fmincon_target,x0,[],[],[],[],[-pi/6;-pi/4;-pi],[pi/6;pi/4;pi]);
end
function r = target(x,p_s)
  T = autoGen_fk_space(x(1),x(2),x(3));
  r = norm(p_s - T(1:3,4));
end