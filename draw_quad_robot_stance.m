function[]= draw_quad_robot_stance(fig, state, quad_param, color_factor)
% input state must be 42x1 stance state
    set(0, 'CurrentFigure', fig)
    % draw robot and draw terrain 
    pc        = state(1:3);
    roll_ang  = state(4);
    pitch_ang = state(5);
    yaw_ang   = state(6);

    q = zeros(3,6);
    q(:,1) = state(7:9)';
    q(:,2) = state(10:12)';
    q(:,3) = state(13:15)';
    q(:,4) = state(16:18)';

    F = zeros(3,6);
    F(:,1) = state(19:21)';
    F(:,2) = state(22:24)';
    F(:,3) = state(25:27)';
    F(:,4) = state(28:30)';


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


    % draw body
    draw_cube(fig, t_ec, R_ec,quad_param.x_length,quad_param.y_length,quad_param.z_length,color_factor*[1 0 1])

    % draw legs and force
    % define a draw arrow function
    drawArrow = @(x,y,z,varargin) quiver3( x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),z(2)-z(1),0, varargin{:} );
    for i=1:quad_param.leg_num
        thetalist_link1 =[q(1,i); q(2,i)];
        thetalist_link2 =[q(1,i); q(2,i); q(3,i)];
            T1 = FKinSpace(quad_param.M1, quad_param.Slist_link1, thetalist_link1);
        % link1 pose and orientation
        p_s1 = T1(1:3,4);
        R_s1 = T1(1:3,1:3);
        p_c1 = quad_param.R_cs(:,:,i)*p_s1 + quad_param.t_cs(:,i);
        R_c1 = quad_param.R_cs(:,:,i)*R_s1;
        p_e1 = R_ec*p_c1 + t_ec;
        R_c1 = R_ec*R_c1;
        draw_cube(fig, p_e1, R_c1,quad_param.leg_l1,quad_param.limb_width,quad_param.limb_width,color_factor*[0.2 0.6 1])

        % link2 pose and orientation
        T2 = FKinSpace(quad_param.M2, quad_param.Slist_link2, thetalist_link2);
        p_s2 = T2(1:3,4);
        R_s2 = T2(1:3,1:3);
        p_c2 = quad_param.R_cs(:,:,i)*p_s2 + quad_param.t_cs(:,i);
        R_c2 = quad_param.R_cs(:,:,i)*R_s2;
        p_e2 = R_ec*p_c2 + t_ec;
        R_c2 = R_ec*R_c2;
        draw_cube(fig, p_e2, R_c2,quad_param.leg_l2,quad_param.limb_width,quad_param.limb_width,color_factor*[0 0 1])

        T3 = FKinSpace(quad_param.M3, quad_param.Slist_link2, thetalist_link2);
        p_s3 = T3(1:3,4);
        R_s3 = T3(1:3,1:3);
        p_c3 = quad_param.R_cs(:,:,i)*p_s3 + quad_param.t_cs(:,i);
        p_e3 = R_ec*p_c3 + t_ec;
        force_scale_factor = 0.009;
        x_arrow = [p_e3(1) p_e3(1)+force_scale_factor*F(1,i)];
        y_arrow = [p_e3(2) p_e3(2)+force_scale_factor*F(2,i)];
        z_arrow = [p_e3(3) p_e3(3)+force_scale_factor*F(3,i)];
        drawArrow(x_arrow,y_arrow,z_arrow,'linewidth',2,'color',color_factor*[1 0 0])
    end
end