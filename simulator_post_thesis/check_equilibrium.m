function [R_eq,i,fail] = check_equilibrium(previous_rotation_matrix_eye,previous_omega_eye,sampling_time,theta,k)
max_timesteps =5000;
% initializations
ext_eye_torque = [0;0;0];
omega_head = [0;0;0];
omega_eye = [0;0;0];
aux_vel_eye=zeros(3,max_timesteps);
inertia_tensor_eye = 0.1*[ 0.0004759 0 0; 0 0.0004316 0;0 0 0.0003956];
i = 1;
D_eye = 2*[0.02 0 0; 0 0.02 0; 0 0 0.02];
dimension = 'prototype';
R_eq = zeros(3,3);
fail = 0;
while(1) % number of iterations
    
    %computing EYE alpha and omega;
    inertia_tensor_eye_new = inertia_tensor_eye;%previous_rotation_matrix_eye'*inertia_tensor_eye*previous_rotation_matrix_eye;
    alpha_eye = (inertia_tensor_eye_new)\(ext_eye_torque - cross(omega_eye,inertia_tensor_eye_new*omega_eye));
    omega_eye = previous_omega_eye+sampling_time*alpha_eye;
    aux_vel_eye(:,i)= omega_eye;
    previous_omega_eye = omega_eye;
    omega_eye_hat = [0 -omega_eye(3) omega_eye(2); omega_eye(3) 0 -omega_eye(1); -omega_eye(2) omega_eye(1) 0];
    %computing rotation matrix for the EYE
    
    rotation_matrix_eye = expm(sampling_time*omega_eye_hat);
    rotation_eye_in_head = previous_rotation_matrix_eye*rotation_matrix_eye;
    previous_rotation_matrix_eye = rotation_eye_in_head;
    
    eul_ang = rotm2eul(rotation_eye_in_head);
    if norm(eul_ang)>60*pi/180
       fail=2;
       break;
    end
    
    % computing torques for the eye and head
    [ext_eye_torque,~,~,~] = compute_eye_torques2(rotation_eye_in_head,omega_eye,theta,k,omega_head,D_eye,dimension);   %floor((i-1)/10)+1
    
    if i~=1 && norm(aux_vel_eye(:,i) - aux_vel_eye(:,i-1)) < 0.0001 && norm(aux_vel_eye(:,i)) < 0.0001 %&  norm(alpha_eye) < 0.000008
        if norm(ext_eye_torque) < 0.001
            R_eq(:,:) = rotation_eye_in_head;
            break;
        end
        
    end
    
    %get quaternion from rotation matrix
    %     q_eye = rotm2quat(rotation_eye_in_head);
    %get quaternion matrix to individual angles
    %     r_x_eye = q_eye(2)/q_eye(1);
    %     r_y_eye = q_eye(3)/q_eye(1);
    %     r_z_eye = q_eye(4)/q_eye(1);
    
    if i > max_timesteps
        fail =  1;
        break;
    end
    i = i+1;
end
end