function [tau,tau_g] = build_jacobian(P,theta,force,n_points,v)
%P is the matrix for the insertion points (in world frame) dim-> 6x3
%theta is a vector for the angles of the joints
%f is a matrix for the forces applied at the insertion points -- for now assume its applied on the head (to find the insertion points on the head)
%nt->tilt joint for the neck-torsion
%nr->roll joint for the neck-vertical
%ny-> yaw joint for the neck-horizontal

%Inicializations

l = 0.201;
mass = [2 0.5]; %mass for each body part(with the exception of the eye)
g = 9.8; %gravity acceleration
CoM_neck = [0 -0.201422/2 0];%Approximate center of mass/gravity for neck in world frame
head_CoM_to_CoR= [0 -0.0593695 0];
tau = zeros(4,n_points); %4 joints vs number of insertion points
l_head = P(1:6,:); %distance from each insertion point to the head joint(10x3, six head,4 neck)
l_neck = P(7:10,:);
point_distance = [l_head;l_neck];
tau_g = zeros(4,n_points); %4 joints vs number of insertion points


% Coordinate frames rotation between joints 
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

%Gravity torque computation
f_gw = [0 0 -mass(1)*g];
[f_gh,~] = world_to_head_force(f_gw,theta);
tau_gh = skew(head_CoM_to_CoR)*f_gh;                         

f_gwn = [0 0 -mass(2)*g];
[f_gn,~] = world_to_neck_force(f_gwn,theta);%% + f_gnt <-- try adding this term
tau_gnt =  skew(CoM_neck)*f_gn;
f_gnr = nr_R_nt*f_gn;
tau_gnr = nr_R_nt*tau_gnt;
f_gny = ny_R_nr*f_gnr;
tau_gny = ny_R_nr*tau_gnr;
tau_g = [tau_gh(3);tau_gnt(3);tau_gnr(3);tau_gny(3)];

for i = 1:n_points
    if i > 6
        tau_h = zeros(3,1);
    else
        tau_h = skew(point_distance(i,:))*force(i)*v(:,i);
    end
    
    f_nt = nt_R_h*force(i)*v(:,i);
   
    if i > 6
        tau_nt = nt_R_h*tau_h-skew(point_distance(i,:))*f_nt;
    else
        tau_nt = nt_R_h*tau_h-skew([0 l 0])*f_nt;
    end
    
    f_nr = nr_R_nt*f_nt;
    tau_nr = nr_R_nt*tau_nt;

    f_ny = ny_R_nr*f_nr;
    tau_ny = ny_R_nr*tau_nr;

    tau(:,i) = [tau_h(3);tau_nt(3);tau_nr(3);tau_ny(3)]; 
end
