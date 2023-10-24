function [tau_eye, delta_l, f_norm, flag, DeadZone,tau_muscles,tau_k] = compute_eye_torques2(rotation_matrix_to_head_ref, omega_eye,theta,k,omega_head,D_eye,dimension)

d = [0.04,0.04,0.04,0.04,0.065,0.065]; %distance from the spindle to the motors
DeadZone = 0; %Force velocity to zero, if desired
%motor spindle radius
r = 0.024;% radius of the spindles in m
a = [1 0 0; 0 cos(-90*pi/180) -sin(-90*pi/180); 0 sin(-90*pi/180) cos(-90*pi/180)]; %rotate the insertion points to the right coordinate frame


if strcmp(dimension,'prototype') == 1
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
    
elseif strcmp(dimension,'real') == 1
    %insertion points on the eye
    Q1_0 = a*[-0.002; 0.040; -0.0014]*0.4; %IR in m
    Q2_0 = a*[0.0077; 0; 0.0393]*0.4; %MR in m
    Q3_0 = a*[-0.002; -0.040; 0.0008]*0.4; %SR in m
    Q4_0 = a*[0.0077; 0; -0.0393]*0.4; %LR in m
    Q5_0 = a*[-0.0112; 0.0383; -0.0014]*0.4; %IO in m
    Q6_0 = a*[-0.0118; -0.0381; -0.0012]*0.4; %SO in m
    
    %Muscle initial point
    I1 = a*[-0.1001; 0.0078; 0.0407]*0.4; %IR in m
    I2 = a*[-0.1001; -0.0035; 0.0516]*0.4; %MR in m
    I3 = a*[-0.1001; -0.0149; 0.0407]*0.4; %SR in m
    I4 = a*[-0.1001; -0.0035; 0.0296]*0.4; %LR in m
    I5 = a*[0.045; 0.062; 0.0375]*0.4; %IO in m
    I6 = a*[0.045; -0.062; 0.0375]*0.4; %SO in m
    
end

I = [I1 I2 I3 I4 I5 I6];
Q_0 = [Q1_0 Q2_0 Q3_0 Q4_0 Q5_0 Q6_0];


% Transforming the start points to head reference frame
I_e = rotation_matrix_to_head_ref'*I;


% get insertion points of the muscles in the head ref (Rotation matrix will be eye in head)
Q = rotation_matrix_to_head_ref*Q_0;

% current string length
l = vecnorm(I-Q) + d + r*theta;

% initial string length
l0 = vecnorm(I-Q_0) + d;

%check for slack
delta_l = l-l0;
flag=int8(delta_l<0);

% if any(delta_l<0)
%     disp('WARNING WARNING! NEGATIVE VALUES');
%     disp(delta_l<0);
% end

%get vector direction
length_vec=I_e-Q_0;
length_vec_norm=vecnorm(length_vec);
v1=length_vec(:,1)/length_vec_norm(1);
v2=length_vec(:,2)/length_vec_norm(2);
v3=length_vec(:,3)/length_vec_norm(3);
v4=length_vec(:,4)/length_vec_norm(4);
v5=length_vec(:,5)/length_vec_norm(5);
v6=length_vec(:,6)/length_vec_norm(6);

% Force on each muscle
F1 = ((k*max(0,delta_l(1)))/l0(1))*v1;
F2 = ((k*max(0,delta_l(2)))/l0(2))*v2;
F3 = ((k*max(0,delta_l(3)))/l0(3))*v3;
F4 = ((k*max(0,delta_l(4)))/l0(4))*v4;
F5 = ((k*max(0,delta_l(5)))/l0(5))*v5;
F6 = ((k*max(0,delta_l(6)))/l0(6))*v6;
% F1 = ((k*delta_l(1))/l0(1))*v1;
% F2 = ((k*delta_l(2))/l0(2))*v2;
% F3 = ((k*delta_l(3))/l0(3))*v3;
% F4 = ((k*delta_l(4))/l0(4))*v4;
% F5 = ((k*delta_l(5))/l0(5))*v5;
% F6 = ((k*delta_l(6))/l0(6))*v6;
f_norm=vecnorm([F1 F2 F3 F4 F5 F6]);
f = [F1 F2 F3 F4 F5 F6];

% tau=torque computation
tau1 = cross(Q1_0,F1);
tau2 = cross(Q2_0,F2);
tau3 = cross(Q3_0,F3);
tau4 = cross(Q4_0,F4);
tau5 = cross(Q5_0,F5);
tau6 = cross(Q6_0,F6);

%elastic torque
tau_muscles = tau1 + tau2 + tau3 + tau4 + tau5 + tau6;
tau_k = vecnorm([tau1 tau2 tau3 tau4 tau5 tau6]);
%Damping torque

tau_friction = -D_eye*(omega_eye - rotation_matrix_to_head_ref'*omega_head);

%full eye torque computation
tau_eye = tau_muscles + tau_friction;

%stop condition
static_friction_torque = 0.005;
if norm(omega_eye)< 0.01
    if norm(tau_eye) < static_friction_torque
        tau_eye = zeros(3,1); %instead put omega_eye = 0
        %         DeadZone = 1;
        %         bp = 1;
        %         bp
    end
end