function [n_point,matrix] = world_to_neck_force(point,theta)

[~,w_T_n] = neck_to_world_force(point,theta); %transformation from world to neck
w_R_n = w_T_n(1:3,1:3);
matrix = w_R_n';
n_point = matrix*point';


