function n_point = world_to_neck(point,theta)

[~,w_T_n] = neck_to_world(point,theta); %transformation from world to neck
w_R_n = w_T_n(1:3,1:3);
w_d_n = w_T_n(1:3,4);

n_T_w = [w_R_n' -w_R_n'*w_d_n;zeros(1,3) 1];% transformation from neck to world
matrix = n_T_w(1:3,1:3);
n_point_aux = n_T_w*[point';1];
n_point = n_point_aux(1:3);

