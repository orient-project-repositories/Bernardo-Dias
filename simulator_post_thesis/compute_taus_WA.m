 function [f,tau_k,F1,F2,F3,F4,F5,F6, flag, delta_l,slack,tau_muscles,tau_friction,t_eye,ins_point] = compute_taus_WA(rotation_matrix_to_head_ref, omega_eye,theta,k,omega_head,D_eye,dimension)
 
d = [0.04,0.04,0.04,0.04,0.065,0.065];
slack = zeros(1,12);

%motor spindle radius 
r = 0.024;% in m
r_eye = 0.079/2;
a = [1 0 0; 0 cos(-90*pi/180) -sin(-90*pi/180); 0 sin(-90*pi/180) cos(-90*pi/180)];

%insertion points on the eye
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

l0 = zeros(6,1);
t_eye_0=tangent_points_eye(I,Q_0);
[~,alpha_0] = plot_arc(t_eye_0,Q_0,10);

for j=1:6
    length_head_to_tangent = vecnorm(I(:,j) - t_eye_0(:,j));
    length_head_to_eye = vecnorm(I(:,j) - Q_0(:,j));
    if length_head_to_eye < length_head_to_tangent
        l0(j) = length_head_to_eye + d(j);
    else
        l0(j) = length_head_to_tangent + d(j) + r_eye*alpha_0(j);
    end
end

% Transforming the start points to eye reference frame
I_e = rotation_matrix_to_head_ref'*I;


% get insertion points of the muscles in the head ref (Rotation matrix will be eye in head)
Q = rotation_matrix_to_head_ref*Q_0;

% get insertion points in the head written in the world frame
I_w = I;

% get insertion points in the eye written in the world frame
Q_w = Q;

% get tangent points to evaluate how we are going to compute the forces
t_eye=tangent_points_eye(I_w,Q_w);
[~,alpha] = plot_arc(t_eye,Q_w,10);
F=zeros(3,6);
f_norm=zeros(6,1);
tau_muscles=zeros(3,1);
flag_arc=zeros(1,6);
l=zeros(6,1);
for j=1:6
    length_head_to_tangent = vecnorm(I_w(:,j) - t_eye(:,j));
    length_head_to_eye = vecnorm(I_w(:,j) - Q_w(:,j));
    if length_head_to_eye < length_head_to_tangent % no wrap around
        % this flag is used to know if the side slip should be computed or not
        flag_arc(j)=0;
        ins_point(:,j) = Q(:,j);
        % current string length
        l(j) = vecnorm(I(:,j)-Q(:,j)) + d(j) + r*theta(j);
        delta_l = l(j)-l0(j);
        flag=int8(delta_l<0);
        
        %get vector direction
        length_vec=I_e(:,j)-Q_0(:,j);
        length_vec_norm=vecnorm(length_vec);
        v=length_vec/length_vec_norm;
        
        %muscle force and torque 
        F(:,j)=((k*max(0,delta_l))/l0(j))*v;
        f_norm(j)=vecnorm(F(:,j));        
        tau = cross(rotation_matrix_to_head_ref'*Q_w(:,j),F(:,j));
        tau_m(j) = norm(tau);
    else % wrap around
        flag_arc(j)=1;
        ins_point(:,j) = t_eye(:,j);
        % current string length
        l(j) = vecnorm(I_w(:,j)-t_eye(:,j)) + d(j) + r*theta(j) + r_eye*alpha(j);
        delta_l = l(j)-l0(j);
        flag=int8(delta_l<0);
        
        %get vector direction
        length_vec=I_e(:,j)-rotation_matrix_to_head_ref'*t_eye(:,j);
        length_vec_norm=vecnorm(length_vec);
        v=length_vec/length_vec_norm;
        
        %muscle force and torque
        F(:,j)=((k*max(0,delta_l))/l0(j))*v;
        f_norm(j)=vecnorm(F(:,j)); 
        tau = cross(rotation_matrix_to_head_ref'*t_eye(:,j),F(:,j));
        tau_m(j) = norm(tau);
    end
    
    tau_muscles=tau_muscles+tau;
    
end
tau_k = tau_m;
%Damping torque

tau_friction = -D_eye*(omega_eye - rotation_matrix_to_head_ref'*omega_head);%-D*omega_eye;%

%full eye torque computation
tau_eye = tau_muscles + tau_friction;

F1 = F(:,1);
F2 = F(:,2);
F3 = F(:,3);
F4 = F(:,4);
F5 = F(:,5);
F6 = F(:,6);
f = vecnorm(F);
%stop condition
%  static_friction_torque = 0.01;
%  if norm(omega_eye)< 0.008
%      if norm(tau_eye) < static_friction_torque
%          tau_eye = zeros(3,1);      
%      end
%  end

