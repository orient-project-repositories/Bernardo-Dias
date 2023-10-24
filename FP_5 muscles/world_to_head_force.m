function [h_point,matrix] = world_to_head_force(point,theta)

[~,w_T_h] = head_to_world_force(point,theta); 
w_R_h = w_T_h(1:3,1:3);
matrix = w_R_h';
h_point = matrix*point';


