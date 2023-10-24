function tau_head = compute_tau_head (omega_eye, omega_head, R_head_in_world, R_gaze, R_eye_in_head,tau_ext,theta,k) %torque de controlo como argumento

d_eye = [0.02 0 0; 0 0.02 0; 0 0 0.02];
mass = 4.38;
d = [0.04,0.04,0.04,0.04,0.065,0.065];
r = 0.024;% in m
a = [1 0 0; 0 0 -sin(-90*pi/180); 0 sin(-90*pi/180) 0];

%Muscle initial point 
I1 = a*[-0.1001; 0.0078; 0.0407]; %IR in m
I2 = a*[-0.1001; -0.0035; 0.0516]; %MR in m
I3 = a*[-0.1001; -0.0149; 0.0407]; %SR in m
I4 = a*[-0.1001; -0.0035; 0.0296]; %LR in m
I5 = a*[0.045; 0.062; 0.0375]; %IO in m
I6 = a*[0.045; -0.062; 0.0375]; %SO in m

I = [I1 I2 I3 I4 I5 I6];

Q1_0 = a*[-0.002; 0.040; -0.0014]; %IR in m
Q2_0 = a*[0.0077; 0; 0.0393]; %MR in m
Q3_0 = a*[-0.002; -0.040; 0.0008]; %SR in m 
Q4_0 = a*[0.0077; 0; -0.0393]; %LR in m
Q5_0 = a*[-0.0112; 0.0383; -0.0014]; %IO in m
Q6_0 = a*[-0.0118; -0.0381; -0.0012]; %SO in m

Q_0 = [Q1_0 Q2_0 Q3_0 Q4_0 Q5_0 Q6_0];

l1 = norm(I1-R_eye_in_head*Q1_0) + d(1) + r*(theta(1)) ;
l2 = norm(I2-R_eye_in_head*Q2_0) + d(2) + r*(theta(2)) ;
l3 = norm(I3-R_eye_in_head*Q3_0) + d(3) + r*(theta(3)) ;
l4 = norm(I4-R_eye_in_head*Q4_0) + d(4) + r*(theta(4)) ;
l5 = norm(I5-R_eye_in_head*Q5_0) + d(5) + r*(theta(5)) ;
l6 = norm(I6-R_eye_in_head*Q6_0) + d(6) + r*(theta(6)) ;

l1_0 = norm(I1-Q1_0) + d(1);%0.151502735392456;
l2_0 = norm(I2-Q2_0) + d(2);%0.148555884225591;
l3_0 = norm(I3-Q3_0) + d(3);%0.148837631359746;
l4_0 = norm(I4-Q4_0) + d(4);%0.16798554605892;
l5_0 = norm(I5-Q5_0) + d(5);%0.137341827458255;
l6_0 = norm(I6-Q6_0) + d(6);%0.137767712620365;

l = [l1 l2 l3 l4 l5 l6];
l0 = [l1_0 l2_0 l3_0 l4_0 l5_0 l6_0];

%get vector direction
v1 = -(I1-R_eye_in_head*Q1_0)/norm(I1-R_eye_in_head*Q1_0);
v2 = -(I2-R_eye_in_head*Q2_0)/norm(I2-R_eye_in_head*Q2_0);
v3 = -(I3-R_eye_in_head*Q3_0)/norm(I3-R_eye_in_head*Q3_0);
v4 = -(I4-R_eye_in_head*Q4_0)/norm(I4-R_eye_in_head*Q4_0);
v5 = -(I5-R_eye_in_head*Q5_0)/norm(I5-R_eye_in_head*Q5_0);
v6 = -(I6-R_eye_in_head*Q6_0)/norm(I6-R_eye_in_head*Q6_0);

F1 = ((k*max(0,(l(1)-l0(1))))/l0(1))*v1;
F2 = ((k*max(0,(l(2)-l0(2))))/l0(2))*v2;
F3 = ((k*max(0,(l(3)-l0(3))))/l0(3))*v3;
F4 = ((k*max(0,(l(4)-l0(4))))/l0(4))*v4;
F5 = ((k*max(0,(l(5)-l0(5))))/l0(5))*v5;
F6 = ((k*max(0,(l(6)-l0(6))))/l0(6))*v6;

F_eye2 = [F1 F2 F3 F4 F5 F6];

tau1 = cross(I1,F1);
tau2 = cross(I2,F2);
tau3 = cross(I3,F3);
tau4 = cross(I4,F4);
tau5 = cross(I5,F5);
tau6 = cross(I6,F6);

tau_d = -d_eye* (-R_eye_in_head*omega_eye + omega_head);
tau_eye = tau1 + tau2 + tau3 + tau4 + tau5 + tau6 + tau_d;
%gravitical force
%Fg = mass*[0;0;-9.8];

%center of mass
%cm_head = R_world*[0.012; 0; 0.07];
%cm_head = R_world*[0; 0; 0];0

%Damping matrix
D_neck =2*[0.55 0 0; 0 0.55 0; 0 0 0.55]; %eye(3);
%D_head = [0.055 0 0; 0 0.055 0; 0 0 0.055];%eye(3);

%Elasticity matrix
%K_neck = eye(3);%0.015 0 0; 0 0.015 0; 0 0 0.015
%K_head = eye(3);

K_neck = 35*[1 0 0; 0 1 0; 0 0 1];
%K_head = [1 0 0; 0 1 0; 0 0 1];

% muscle start point
%I = [0.01;0;0.00]; %0.01;0;0.03

%Insertion points on the head
% Q_0 = [0.46;0;0.00];
%
%Conversion to world reference frame
% Q = R_world*Q_0;
% 
% %intial length
% l = norm(I-Q_0);
% %length of the muscles length = 35 mm;
% L = norm(I-Q);
% 
% %vector direction
% v = (I-Q)/L;
% 
% %compute torques
% F = k/l*max((L-l),0)*v;

%eye torque

    %Damping(eye/head)
    
%     tau_d_head = -D_head*(omega_head-R_eye_in_head*omega_eye);
    
    %Damping(head/neck)
    tau_d_neck = -D_neck*(omega_head);
   
    %Elasticity(eye/head)
    
%     if R_head_in_world == 0 || R_gaze == 0
%         aux_head = zeros(3,3);
%     else 
%         aux_head = logm((R_head_in_world));
%     end
%     a = logm(R_eye_in_head);
%     b = logm(R_gaze);
%     aux_head = a;
%     tau_k_head = -K_head*[aux_head(3,2); aux_head(1,3); aux_head(2,1)];
    
    %Elasticity(head/neck)
    R_head_in_world = R_head_in_world;
    if isequal(diag(R_head_in_world),[0;0;0])
        aux_neck = zeros(3,3);
    else 
        aux_neck = logm(R_head_in_world);
    end
    tau_k_neck = -K_neck*[aux_neck(3,2); aux_neck(1,3); aux_neck(2,1)];
    [aux_neck(3,2); aux_neck(1,3); aux_neck(2,1)];
    
    %Gravitational
    %tau_g = cross(cm_head,Fg);
    
    %external 
%     tau_ext = 0;
%     if norm(tau_d_head) < 0 && norm(tau_d_head) ~= 0
%         tau_ext = -0.3;
%     elseif norm(tau_d_head) > 0 && norm(tau_d_head) ~= 0
%         tau_ext = 0.3;
%     end

%tau_ext = [0.3;0.3;0.3];

%tau_ext = 1;
%External torque(Angular force applied by the muscles);
%tau_ext = cross(Q,F);

%compute total torque (depending on the dynamics)
tau_head = tau_ext  + tau_k_neck; 
tau_head = R_head_in_world'*tau_head  + tau_d_neck + tau_eye;
bp = 1;

%stop condition
static_friction_torque = 0.0000008;
if norm(omega_head) < 0.0000008
    if norm(tau_head) < static_friction_torque
        tau_head = zeros(3,1);
    end
end