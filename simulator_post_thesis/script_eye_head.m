function [final_state,aux_state, aux_vel_eye, aux_gaze1, aux_tau_eye, aux_f_eye,aux_flag,aux_delta] = script_eye_head(previous_rotation_matrix_eye,previous_rotation_matrix_head,previous_omega_eye,previous_omega_head,sampling_time,theta,k,p,u_optimal,saccade_nr,dimension)

% initializations
ext_eye_torque = [0;0;0];
ext_head_torque = [0;0;0];
omega_head = [0;0;0];
omega_eye = [0;0;0];

aux_state = zeros(12,3,p);
aux_tau_eye=zeros(p,3);
aux_f_eye=zeros(p,6);
aux_flag=zeros(p,6);
aux_delta=zeros(p,6);

if strcmp(dimension,'prototype') == 1
    inertia_tensor_eye = 1*[ 0.0004759 0 0; 0 0.0004316 0;0 0 0.0003956];
    inertia_tensor_head = [0.181 0 0; 0  0.215 0; 0 0 0.142];
    D_eye = 2*[0.02 0 0; 0 0.02 0; 0 0 0.02]; %Damping matrix for the eye
    D_head = 3*[0.55 0 0; 0 0.55 0; 0 0 0.55];
elseif strcmp(dimension,'real') == 1
    inertia_tensor_eye = 10^-7*[ 4.759 0 0; 0 4.316 0;0 0 3.956];
    inertia_tensor_head= 10^-3*[0.181 0 0; 0  0.215 0; 0 0 0.142];
    D_eye = 0.002*[0.02 0 0; 0 0.02 0; 0 0 0.02];
    D_head = 0.003*[0.55 0 0; 0 0.55 0; 0 0 0.55];
end

for i=1:p % number of iterations (optimal saccade time)
    
    % State equations
    %state_omega_eye = omega_eye - rotation_eye_in_world'*omega_head;
    %     v_eye = rotation_matrix_gaze*omega_eye_hat;
    %     v_head = rotation_matrix_head*omega_head_hat;
    %     acc_eye = (inertia_tensor_eye_new)\(ext_eye_torque-cross(omega_eye,inertia_tensor_eye_new*omega_eye));
    %     acc_head = (inertia_tensor_head_new)\(ext_head_torque-cross(omega_head,inertia_tensor_head_new*omega_head));
    
    
    %computing EYE alpha and omega;
    inertia_tensor_eye_new = inertia_tensor_eye;%previous_rotation_matrix_eye'*inertia_tensor_eye*previous_rotation_matrix_eye;
    alpha_eye = (inertia_tensor_eye_new)\(ext_eye_torque - cross(omega_eye,inertia_tensor_eye_new*omega_eye)); %acceleration
    aux_acc_eye(i,:) = alpha_eye;
    omega_eye = previous_omega_eye+sampling_time*alpha_eye; %angular velocity
    aux_vel_eye(i,:)= omega_eye;
    previous_omega_eye = omega_eye;
    omega_eye_hat = skew(omega_eye);%[0 -omega_eye(3) omega_eye(2); omega_eye(3) 0 -omega_eye(1); -omega_eye(2) omega_eye(1) 0];
    
    % computing HEAD alpha and omega
    inertia_tensor_head_new = inertia_tensor_head;%previous_rotation_matrix_head'*inertia_tensor_head*previous_rotation_matrix_head;
    alpha_head = (inertia_tensor_head_new)\(ext_head_torque-cross(omega_head,inertia_tensor_head_new*omega_head));
    omega_head = previous_omega_head + sampling_time*alpha_head;
    %     aux_vel_head(i,:) = omega_head;
    previous_omega_head = omega_head;
    omega_head_hat = skew(omega_head);%[0 -omega_head(3) omega_head(2); omega_head(3) 0 -omega_head(1); -omega_head(2) omega_head(1) 0];
    
    
    %computing rotation matrix for the EYE
    rotation_matrix_eye = expm(sampling_time*omega_eye_hat);
    rotation_eye_in_world = previous_rotation_matrix_eye*rotation_matrix_eye;
    previous_rotation_matrix_eye = rotation_eye_in_world;
    
    %computing rotation matrix for the HEAD
    rotation_matrix_head = expm(sampling_time*omega_head_hat);
    rotation_head_in_world = previous_rotation_matrix_head*rotation_matrix_head;
    previous_rotation_matrix_head = rotation_head_in_world;
    
    %computing rotation for the GAZE
    rotation_matrix_gaze = rotation_eye_in_world;
    
    %rotation matrix eye in head
    rotation_eye_in_head= rotation_head_in_world'*rotation_eye_in_world;
    
    % computing torques for the eye and head
    [ext_eye_torque, aux_delta(i,:), aux_f_eye(i,:), aux_flag(i,:),DeadZone,tau_m,tau_f] = compute_eye_torques2(rotation_eye_in_head, omega_eye,theta+u_optimal(i,1:6),k,omega_head,D_eye,dimension);
    %     if DeadZone
    %         previous_omega_eye = zeros(3,1);
    %         i
    %     end
    tau_muscles(i,:) = tau_m;
    tau_friction(i,:) = tau_f;
    %    [ext_head_torque,~] = compute_tau_head(omega_eye, omega_head,rotation_head_in_world, rotation_matrix_gaze,rotation_eye_in_head,u_head1+ u_optimal(i,7:9)' ,theta+u_optimal(i,1:6),k);%
    
    %     get quaternion from rotation matrix
    q_eye = rotm2quat(rotation_eye_in_world);
    q_head = rotm2quat(rotation_head_in_world);
    
    q_gaze = rotm2quat(rotation_matrix_gaze); %orientation in quaternion
    rot_vec = quat2rod(q_gaze); %orientation in rotation vector
    aux_gaze(i,:) = q_gaze;
    aux_gaze1(i,:) = rot_vec;
    
    aux_tau_eye(i,:) = ext_eye_torque;
    %     aux_tau_head(i,:) = ext_head_torque;
    
    %     q_head = rotm2quat(rotation_head_in_world);
    %     q_head1 = quat2rod(q_head);
    %     aux_head(i,:) = q_head;
    %     aux_head1(i,:) = q_head1;
    
    aux_state(:,:,i) = [rotation_matrix_gaze; rotation_matrix_head ; diag(omega_eye); diag(omega_head)];
    
    
end

final_state = [rotation_matrix_gaze; rotation_matrix_head ; diag(omega_eye); diag(omega_head)];

% plots for eye and head position, velocity and muscle forces

% x = linspace(0,i,i);
%
% figure();
% plot(x,aux_f_eye(:,1),'Color',[0,0.4470,0.7410]);
% grid on
% grid minor
% hold on
% plot(x,aux_f_eye(:,2),'Color',[0.8500 0.3250 0.0980]);
% plot(x,aux_f_eye(:,3),'Color',[0.9290 0.6940 0.1250]);
% plot(x,aux_f_eye(:,4),'Color',[0.4940 0.1840 0.5560]);
% plot(x,aux_f_eye(:,5),'Color',[0.4660 0.6740 0.1880]);
% plot(x,aux_f_eye(:,6),'Color',[0.3010 0.7450 0.9330]);
% % plot(x,1*aux_flag(:,1),'.','Color',[0,0.4470,0.7410]);
% % plot(x,2*aux_flag(:,2),'.','Color',[0.9290 0.6940 0.1250]);
% % plot(x,3*aux_flag(:,3),'.','Color',[0.8500 0.3250 0.0980]);
% % plot(x,4*aux_flag(:,4),'.','Color',[0.4940 0.1840 0.5560]);
% % plot(x,5*aux_flag(:,5),'.','Color',[0.4660 0.6740 0.1880]);
% % plot(x,6*aux_flag(:,6),'.','Color',[0.3010 0.7450 0.9330]);
% xlabel('time');
% ylabel('muscle forces');
% legend("IR","MR","SR","LR","IO","SO");

% figure();
% hold on
% plot(x,aux_vel_eye);
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('eye velocity(rad/s)');
% legend("eye theta x", "eye theta y", "eye theta z");

% figure();
% plot(x,aux_vel_head);
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('head angular velocity (rad/s)');
% legend("head omega x", "head omega y", "head omega z");
%
% figure();
% hold on
% plot(x,aux_tau_eye);
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('eye torque (N.m)');
% legend("torsion", "horizontal", "vertical");
%
% figure();
% hold on
% plot(x,tau_muscles);
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('eye torque (N.m)');
% legend("torsion", "horizontal", "vertical");
%
% figure();
% hold on
% plot(x,tau_friction);
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('eye torque (N.m)');
% legend("torsion", "horizontal", "vertical");
% figure();
% plot(x,aux_tau_head);
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('head torque (N.m)');
% legend("torsion", "horizontal", "vertical");


