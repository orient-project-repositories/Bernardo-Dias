function h_point = world_to_head(point,theta)

[~,w_T_h] = head_to_world(point,theta); %transformation from world to neck
w_R_h = w_T_h(1:3,1:3);
w_d_h = w_T_h(1:3,4);

h_T_w = [w_R_h' -w_R_h'*w_d_h; zeros(1,3) 1];
matrix = h_T_w(1:3,1:3);
h_point_aux = h_T_w*[point';1];
h_point = h_point_aux(1:3);



