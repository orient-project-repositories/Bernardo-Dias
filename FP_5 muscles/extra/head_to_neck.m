function [n_point,matrix] = head_to_neck(point,theta)

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

%matrix = nt_T_h;
h_point_aux = matrix*[point;1];
n_point = h_point_aux(1:3);
