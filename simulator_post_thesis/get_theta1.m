function [c,ceq] = get_theta1(theta,R,k) % torque computation, for theta minimization
r = 0.024;

a = [1 0 0; 0 cos(-90*pi/180) -sin(-90*pi/180); 0 sin(-90*pi/180) cos(-90*pi/180)]; % transformation into the right coordinate frame (neuroscience CF)
% transformation into the right coordinate frame

d = [0.04,0.04,0.04,0.04,0.065,0.065];
F=zeros(3,6);
f_norm=zeros(6,1);
tau_muscles=zeros(3,1);
c = [];
l=zeros(6,1);
l0=zeros(6,1);
% %insertion points on the eye
Q1_0 = a*[-0.002; 0.040; -0.0014]; %IR in m
Q2_0 = a*[0.0077; 0; 0.0393]; %MR in m
Q3_0 = a*[-0.002; -0.040; 0.0008]; %SR in m 
Q4_0 = a*[0.0077; 0; -0.0393]; %LR in m
Q5_0 = a*[-0.0112; 0.0383; -0.0014]; %IO in m
Q6_0 = a*[-0.0118; -0.0381; -0.0012]; %SO in m
%Muscle initial point 
I1 = a*[-0.1001; 0.0078; 0.0407]; %IR in m
I2 = a*[-0.1001; -0.0035; 0.0516]; %MR in m
I3 = a*[-0.1001; -0.0149; 0.0407]; %SR in m
I4 = a*[-0.1001; -0.0035; 0.0296]; %LR in m
I5 = a*[0.045; 0.062; 0.0375]; %IO in m
I6 = a*[0.045; -0.062; 0.0375]; %SO in m

Q_0 = [Q1_0 Q2_0 Q3_0 Q4_0 Q5_0 Q6_0];
I = [I1 I2 I3 I4 I5 I6];
I_e = R'*I;


% get insertion points of the muscles in the head ref (Rotation matrix will be eye in head)
Q = R*Q_0;

% get insertion points in the head written in the world frame 
%Assuming head and world have the same ref frame
I_w =I;

% get insertion points in the eye written in the world frame
Q_w = Q;

% I1_e = R'*I1;
% I2_e = R'*I2;
% I3_e = R'*I3;
% I4_e = R'*I4;
% I5_e = R'*I5;
% I6_e = R'*I6;
% 
% v1 = (I1_e-Q1_0)/norm(I1_e-Q1_0);
% v2 = (I2_e-Q2_0)/norm(I2_e-Q2_0);
% v3 = (I3_e-Q3_0)/norm(I3_e-Q3_0);
% v4 = (I4_e-Q4_0)/norm(I4_e-Q4_0);
% v5 = (I5_e-Q5_0)/norm(I5_e-Q5_0);
% v6 = (I6_e-Q6_0)/norm(I6_e-Q6_0);
% 
% l1 = norm(I1-R*Q1_0) + d(1) + r*theta(1) ;
% l2 = norm(I2-R*Q2_0) + d(2) + r*theta(2) ;
% l3 = norm(I3-R*Q3_0) + d(3) + r*theta(3) ;
% l4 = norm(I4-R*Q4_0) + d(4) + r*theta(4) ;
% l5 = norm(I5-R*Q5_0) + d(5) + r*theta(5) ;
% l6 = norm(I6-R*Q6_0) + d(6) + r*theta(6) ;
% 
% l1_0 = norm(I1-Q1_0) + d(1);%0.151502735392456;
% l2_0 = norm(I2-Q2_0) + d(2);%0.148555884225591;
% l3_0 = norm(I3-Q3_0) + d(3);%0.148837631359746;
% l4_0 = norm(I4-Q4_0) + d(4);%0.16798554605892;
% l5_0 = norm(I5-Q5_0) + d(5);%0.137341827458255;
% l6_0 = norm(I6-Q6_0) + d(6);%0.137767712620365;
% 
% 
% l = [l1 l2 l3 l4 l5 l6];
% l0 = [l1_0 l2_0 l3_0 l4_0 l5_0 l6_0];
% 
% c = -[(k/l1_0)*(l1-l1_0) (k/l2_0)*(l2-l2_0) (k/l3_0)*(l3-l3_0) (k/l4_0)*(l4-l4_0) (k/l5_0)*(l5-l5_0) (k/l6_0)*(l6-l6_0)] ;
% 
% F1 = ((k*max(0,(l(1)-l0(1))))/l0(1))*v1;
% F2 = ((k*max(0,(l(2)-l0(2))))/l0(2))*v2;
% F3 = ((k*max(0,(l(3)-l0(3))))/l0(3))*v3;
% F4 = ((k*max(0,(l(4)-l0(4))))/l0(4))*v4;
% F5 = ((k*max(0,(l(5)-l0(5))))/l0(5))*v5;
% F6 = ((k*max(0,(l(6)-l0(6))))/l0(6))*v6;
% 
% F_eye_min = [F1 F2 F3 F4 F5 F6];
% 
% tau1 = cross(Q1_0,F1);
% tau2 = cross(Q2_0,F2);
% tau3 = cross(Q3_0,F3);
% tau4 = cross(Q4_0,F4);
% tau5 = cross(Q5_0,F5);
% tau6 = cross(Q6_0,F6);
for j=1:6
    
    length_head_to_eye = vecnorm(I(:,j) - Q_0(:,j));
    
    l0(j) = length_head_to_eye + d(j);
end

for j = 1:6
       
      l(j) = vecnorm(I(:,j)-Q(:,j)) + d(j) + r*theta(j);
        
%         % initial string length
%         l0 = vecnorm(I(:,j)-Q_0(:,j)) + d(j);
        
        delta_l = l(j)-l0(j);
        
        
        %get vector direction
        length_vec=I_e(:,j)-Q_0(:,j);
        length_vec_norm=vecnorm(length_vec);
        v=length_vec/length_vec_norm;
        
        %force on the muscle
        F(:,j)=((k*max(0,delta_l))/l0(j))*v;

        f_norm(j)=vecnorm(F(:,j));
        
        tau = cross(R'*Q_w(:,j),F(:,j));
        c(j) = -(k/l0(j))*delta_l;
         tau_muscles=tau_muscles+tau;
end
ceq = tau_muscles;
