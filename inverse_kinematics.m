function [inv_s,inv_u,inv_k] = inverse_kinematics(p, body)
    % from center of the shoulder
    x = p(1);
    y = p(2);
    z = p(3);
    l1 = body.upper_length;
    l2 = body.lower_length;
    
    tmp = y/sqrt(y*y+z*z);
    if tmp > 1
        tmp = 1;
    elseif tmp < -1
        tmp = -1;
    end
    inv_s = asin(tmp);
    tmp = (x*x+y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);
    if tmp > 1
        tmp = 1;
    elseif tmp < -1
        tmp = -1;
    end
    inv_k = -acos(tmp); % always assume knee is minus

    if abs(inv_k) > 1e-4
        if abs(inv_s) > 1e-4
            temp1 = y/sin(inv_s)+(l2*cos(inv_k)+l1)*x/l2/sin(inv_k);
        else
            temp1 = -z/cos(inv_s)+(l2*cos(inv_k)+l1)*x/l2/sin(inv_k);
        end
        temp2 = -l2*sin(inv_k)-(l2*cos(inv_k)+l1)*(l2*cos(inv_k)+l1)/(l2*sin(inv_k));
        tmp = temp1/temp2;
        if tmp > 1
            tmp = 1;
        elseif tmp < -1
            tmp = -1;
        end
        inv_u = asin(tmp);
    else
        tmp = x/(-l1-l2);
        if tmp > 1
            tmp = 1;
        elseif tmp < -1
            tmp = -1;
        end
        inv_u = asin(tmp);
    end

end