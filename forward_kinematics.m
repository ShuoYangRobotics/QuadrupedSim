function end_effector = forward_kinematics(s, u, k, body)
    % from center of the shoulder, 
    l1 = body.upper_length;
    l2 = body.lower_length;
    end_effector = [ - l2*sin(k + u)       -  l1*sin(u);
                     sin(s)*(l2*cos(k + u) +  l1*cos(u));
                    -cos(s)*(l2*cos(k + u) +  l1*cos(u))   ];
end