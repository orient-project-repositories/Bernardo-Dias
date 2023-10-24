function [w_point,matrix] = neck_to_world(point,theta)

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

matrix = w_T_ny*ny_T_nr*nr_T_nt;
w_point_aux = matrix*[point';1];
w_point = w_point_aux(1:3);
