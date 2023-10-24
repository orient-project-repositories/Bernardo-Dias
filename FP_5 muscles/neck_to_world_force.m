function [w_point,matrix] = neck_to_world_force(point,theta)

l = 0.201; %distance from neck joint to head joint

%Homogeneous transformations
w_T_ny = [cos(theta(4)) -sin(theta(4)) 0 0;
          sin(theta(4)) cos(theta(4)) 0 0;
          0 0 1 0; 
          0 0 0 1];
      
ny_T_nr = [cos(theta(3)) -sin(theta(3)) 0 0;
           0 0 1 0;
           -sin(theta(3)) -cos(theta(3)) 0 0;
           0 0 0 1];
       
nr_T_nt = [0 0 1 0;
           sin(theta(2)) cos(theta(2)) 0 0;
           -cos(theta(2)) sin(theta(2)) 0 0;
           0 0 0 1];

nt_T_h = [0 0 -1 0;
          sin(theta(1)) cos(theta(1)) 0 -l;
          cos(theta(1)) -sin(theta(1)) 0 0;
          0 0 0 1];
nt_R_h = nt_T_h(1:3,1:3);
nr_R_nt = nr_T_nt(1:3,1:3);
ny_R_nr = ny_T_nr(1:3,1:3);
w_R_ny = w_T_ny(1:3,1:3);
matrix = w_R_ny*ny_R_nr*nr_R_nt;
w_point = matrix*point';

