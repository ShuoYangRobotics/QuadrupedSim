function[]= draw_cube(fig, center_t, center_R,L,W,H,rgb)
center1 = center_t(1);
center2 = center_t(2);
center3 = center_t(3);

X=([-L/2, L/2, L/2, -L/2]);
Y=([-W/2, -W/2, W/2, W/2]);
for i=1:4
T_u(:,i)=center_R*[X(i); Y(i);  H/2];
T_l(:,i)=center_R*[X(i); Y(i); -H/2];
end
x_lower_left =center1+T_u(1,1);
x_lower_right=center1+T_u(1,2);
x_upper_right=center1+T_u(1,3);
x_upper_left =center1+T_u(1,4);
y_lower_left =center2+T_u(2,1);
y_lower_right=center2+T_u(2,2);
y_upper_right=center2+T_u(2,3);
y_upper_left =center2+T_u(2,4);
z_lower_left =center3+T_u(3,1);
z_lower_right=center3+T_u(3,2);
z_upper_right=center3+T_u(3,3);
z_upper_left =center3+T_u(3,4);
x_coor1=[x_lower_left x_lower_right x_upper_right x_upper_left x_lower_left ];
y_coor1=[y_lower_left y_lower_right y_upper_right y_upper_left y_lower_left ];
z_coor1=[z_lower_left z_lower_right z_upper_right z_upper_left z_lower_left ];
x_lower_left =center1+T_l(1,1);
x_lower_right=center1+T_l(1,2);
x_upper_right=center1+T_l(1,3);
x_upper_left =center1+T_l(1,4);
y_lower_left =center2+T_l(2,1);
y_lower_right=center2+T_l(2,2);
y_upper_right=center2+T_l(2,3);
y_upper_left =center2+T_l(2,4);
z_lower_left =center3+T_l(3,1);
z_lower_right=center3+T_l(3,2);
z_upper_right=center3+T_l(3,3);
z_upper_left =center3+T_l(3,4);
x_coor2=[x_lower_left x_lower_right x_upper_right x_upper_left x_lower_left ];
y_coor2=[y_lower_left y_lower_right y_upper_right y_upper_left y_lower_left ];
z_coor2=[z_lower_left z_lower_right z_upper_right z_upper_left z_lower_left ];

plot3(x_coor1,y_coor1,z_coor1,'Color',rgb,'LineWidth',3);
plot3(x_coor2,y_coor2,z_coor2,'Color',rgb,'LineWidth',3);

plot3([x_coor2(1) x_coor1(1)],[y_coor2(1) y_coor1(1)],[z_coor2(1) z_coor1(1)],'Color',rgb,'LineWidth',3);
plot3([x_coor2(2) x_coor1(2)],[y_coor2(2) y_coor1(2)],[z_coor2(2) z_coor1(2)],'Color',rgb,'LineWidth',3);
plot3([x_coor2(3) x_coor1(3)],[y_coor2(3) y_coor1(3)],[z_coor2(3) z_coor1(3)],'Color',rgb,'LineWidth',3);
plot3([x_coor2(4) x_coor1(4)],[y_coor2(4) y_coor1(4)],[z_coor2(4) z_coor1(4)],'Color',rgb,'LineWidth',3);

end