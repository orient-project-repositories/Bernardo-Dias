function [final_state,aux_state,aux_tau_eye,u_optimal,aux_gaze1,aux_f_eye,aux_flag,aux_delta,aux_vel_eye,u_with_fixed_end,saccade_ts,aux4,P] = run_optimal_control(goal_rot_vec,R_eye,R_head,omega_eye,omega_head,theta,Ts,k,dimension,extra_time)
% [final_state,aux_state,aux_tau_eye,aux_gaze1, aux_f_eye,aux_flag,aux_delta,aux_vel_eye]
% tic
% ticBytes(gcp);
aux_tau_eye=[];
aux_f_eye=[];
aux_delta=[];
%std_dev = 0.05; %3 degrees
%std_dev = 0.26; %15 degrees
% std_dev = 0.15; %9 degrees
% tempgoal = load('sacc_goal_20_yz.mat');
% all_goals_sequence=tempgoal.allgoals;
% numb = size(all_goals_sequence,2);

% numb = 1;

% fprintf('Progress:\n');
% fprintf(['\n' repmat('.',1,numb) '\n\n']);

C = zeros(3,12);
C(1:3,1:3) = eye(3);
%C(4:6,7:9) = eye(3);
D = zeros(size(C,1),9);

a = [1 0 0; 0 cos(-90*pi/180) -sin(-90*pi/180); 0 sin(-90*pi/180) cos(-90*pi/180)];
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
    
    S = [I1 I2 I3 I4 I5 I6];
    Q = [Q1_0 Q2_0 Q3_0 Q4_0 Q5_0 Q6_0];
    
    I_eye = 1*[ 0.0004759 0 0; 0 0.0004316 0;0 0 0.0003956];
    I_head= [0.181 0 0; 0  0.215 0; 0 0 0.142];
elseif strcmp(dimension,'real') == 1
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
    
    S = [I1 I2 I3 I4 I5 I6];
    Q = [Q1_0 Q2_0 Q3_0 Q4_0 Q5_0 Q6_0];
    
    I_eye = 10^-7*[ 4.759 0 0; 0 4.316 0;0 0 3.956];
    I_head= 10^-3*[0.181 0 0; 0  0.215 0; 0 0 0.142];
end


% all_r = zeros(numb,3);
% per_cent = zeros(numb,1);
% flag = zeros(numb,1);
% flags = zeros(numb,6);
% goal_aux = zeros(numb,3);
% aux3=zeros(numb,1);
% aux4=zeros(numb,1);
% points=zeros(numb,3);
% if_antagonist_muscle=ones(3,numb);
% % if_antagonist_muscle_flag=zeros(1,numb);
% % if_antagonist_muscle_rot=ones(2,numb);
% rot_err=zeros(numb,3);
% nearest_theta_store=zeros(numb,6);


% [all_goals_sequence] = generate_gaussian_saccades( std_dev, numb);
% %all_goals_sequence = [0;0;-0.16];


x = zeros(12,1);

% use this goals to get eta as goal
% for i = 1:numb
%     i
%     goal_rot_vec=all_goals_sequence(:,i)';
    quat = rod2quat(goal_rot_vec);
    goal = quat2rotm(quat);
    goal_eta = rotation_to_vee(R_eye'*goal);
    
    [A,B] = get_jacobian(S,Q,R_eye,R_head,I_eye,I_head,omega_eye,omega_head,theta,k,dimension);%R_eye
    sys = ss(A,B,C,D);
    sys = c2d(sys,Ts);
    
    
    [nearest_eq_theta,eq_rot,rot_err_temp]=lookup_table(goal_rot_vec,theta);
    eq_rot_mat = quat2rotm(rod2quat(eq_rot));
%     rot_err(i,:)=rot_err_temp;
%     nearest_theta_store(i,:) = nearest_eq_theta;
    closest_theta_local = nearest_eq_theta-theta;
    i=1;
    %assuming 1,2 and 5 muscles to be antagonist(bigger theta->agonist)
    %finding antagonists from closest equilibrium theta
    if_antagonist_muscle=ones(3,i);

    if (closest_theta_local(1)>closest_theta_local(3))
        if_antagonist_muscle(1,i)=-1;
    end
    if (closest_theta_local(2)>closest_theta_local(4))
        if_antagonist_muscle(2,i)=-1;
    end
    if (closest_theta_local(5)>closest_theta_local(6))
        if_antagonist_muscle(3,i)=-1;
    end
    
    
    %     %finding antagonists from rx
    %     curr_rot_vec=quat2rod(rotm2quat(R_eye));
    %     diff_rot_vec=goal_rot_vec-curr_rot_vec;
    %     if (diff_rot_vec(1)>0)
    %         if_antagonist_muscle_rot(1,i)=-1;
    %     end
    %     if (diff_rot_vec(2)>0)
    %         if_antagonist_muscle_rot(2,i)=-1;
    %     end
    %
    %     if(if_antagonist_muscle(1:2)-if_antagonist_muscle_rot(1:2)~=0)
    %         if_antagonist_muscle_flag(i)=1;
    %     end

    [u_optimal_final, saccade_dur,P] = optimal_control_with_u_acc(sys,goal_eta,x,closest_theta_local,if_antagonist_muscle(:,i),theta,dimension);

    u_with_fixed_end  = [u_optimal_final';repmat(u_optimal_final(:,end),1,extra_time)'];
    saccade_ts = round(saccade_dur/Ts+extra_time);
    
    [final_state,aux_state,aux_tau_eye,aux_gaze1, ~,aux_flag,~,aux_vel_eye] = visualization(R_eye,R_head,omega_eye,omega_head,Ts,theta,k,saccade_ts,u_with_fixed_end,goal,i,eq_rot_mat,dimension);
%     [final_state,aux_state,aux_tau_eye, aux_f_eye,aux_flag,aux_delta,aux_vel_eye]
u_optimal=u_optimal_final(1:6,:);
temp = rotm2quat(final_state(1:3,:));
all_goals = rotm2eul(goal)*180/pi;
aux4 = norm(quat2eul(temp)*180/pi-all_goals);
end
