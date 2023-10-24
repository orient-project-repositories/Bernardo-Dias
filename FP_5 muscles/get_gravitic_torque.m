function [tau_g,full_gravitic_torque] = get_gravitic_torque(theta)


length = 0.201;
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
          sin(theta(1)) cos(theta(1)) 0 -length;
          cos(theta(1)) -sin(theta(1)) 0 0;
          0 0 0 1];

nt_R_h = nt_T_h(1:3,1:3);
nr_R_nt = nr_T_nt(1:3,1:3);
ny_R_nr = ny_T_nr(1:3,1:3);
w_R_ny = w_T_ny(1:3,1:3);

mass = [2 0.5]; %mass for each body part(with the exception of the eye)
g = 9.8; %gravity acceleration
CoM_neck = [0 0 0.201422/2];%Approximate center of mass/gravity for neck in world frame
l_Com_Cor_h = [0 0 0.05];
%Gravity offset computation
f_gw = [0;0;-mass(1)*g];
f_gh = world_to_head(f_gw',theta);
tau_gh = skew(world_to_head(l_Com_Cor_h,theta))*f_gh;%skew(neck_rotation*CoM_head-head_joint)*f_gh;                          
f_gnt = nt_R_h*f_gh;
f_gwn = [0;0;-mass(2)*g];
f_gn = world_to_neck(f_gwn',theta);% +f_gnt;% + f_gnt <-- try adding this term
tau_gnt =  skew(world_to_neck(CoM_neck,theta))*f_gn;%skew(neck_rotation*CoM_neck)*f_gnt;
f_gnr = nr_R_nt*f_gn;
tau_gnr = nr_R_nt*tau_gnt;
f_gny = ny_R_nr*f_gnr;
tau_gny = ny_R_nr*tau_gnr;
full_gravitic_torque = [tau_gh;tau_gnt;tau_gnr;tau_gny]; 
tau_g = [tau_gh(3);tau_gnt(3);tau_gnr(3);tau_gny(3)];