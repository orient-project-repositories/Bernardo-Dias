function [final_state,aux_state,aux_tau_eye,aux_gaze1, aux_f_eye,aux_flag,aux_delta,aux_vel_eye] = visualization(R_eye,R_head,omega_eye,omega_head,Ts,theta,k,saccade_ts,u,goal,saccade_nr,eq_rot_mat,dimension)

points = zeros(1,70);

a = [1 0 0; 0 0 -sin(-90*pi/180); 0 sin(-90*pi/180) 0];
%Muscle initial point 
I1 = a*[-0.1001; 0.0078; 0.0407]; %IR in m
I2 = a*[-0.1001; -0.0035; 0.0516]; %MR in m
I3 = a*[-0.1001; -0.0149; 0.0407]; %SR in m
I4 = a*[-0.1001; -0.0035; 0.0296]; %LR in m
I5 = a*[0.045; 0.062; 0.0375]; %IO in m
I6 = a*[0.045; -0.062; 0.0375]; %SO in m
% %insertion points on the eye
Q1_0 = a*[-0.002; 0.040; -0.0014]; %IR in m
Q2_0 = a*[0.0077; 0; 0.0393]; %MR in m
Q3_0 = a*[-0.002; -0.040; 0.0008]; %SR in m 
Q4_0 = a*[0.0077; 0; -0.0393]; %LR in m
Q5_0 = a*[-0.0112; 0.0383; -0.0014]; %IO in m
Q6_0 = a*[-0.0118; -0.0381; -0.0012]; %SO in m
I = [I1 I2 I3 I4 I5 I6];
Q = [Q1_0 Q2_0 Q3_0 Q4_0 Q5_0 Q6_0];

arrow_end = [0.515 0 0]';
goal_end = goal*[0.75 0 0]';
eq_end = eq_rot_mat*[0.515 0 0]';

[final_state,aux_state,aux_vel_eye,aux_gaze1,...
    aux_tau_eye, aux_f_eye,aux_flag,aux_delta]...
    = script_eye_head(R_eye,R_head,omega_eye,...
    omega_head,Ts,theta,k,saccade_ts,u,...
    saccade_nr,dimension);

%     x = linspace(0,saccade_ts,saccade_ts);
%     
%     figure();
%     hold on
%     plot(x,aux_vel_eye);
%     grid on
%     grid minor
%     xlabel('time(ms)');
%     ylabel('eye velocity(rad/s)');
%     legend("eye theta x", "eye theta y", "eye theta z");

%rosinit();
% trajectory = zeros(1,7);
% chatpub = rospublisher('/insertion_points','std_msgs/Float64MultiArray'); 
% chatpub1 = rospublisher('/trajectory2','geometry_msgs/Pose');
% msg = rosmessage(chatpub);
% msg1 = rosmessage(chatpub1);
% 
% pause(0.5);
% for i=1:size(aux_state,3)
%     insert1 = aux_state(1:3,:,i)*Q;
%     arrow = aux_state(1:3,:,i)*arrow_end;
% 
%     for j = 1:6
%         points(6*(j-1)+1:6*(j-1)+3) = insert1(:,j);
%         points(6*(j-1)+4:6*(j-1)+6) = I(:,j);
%         len = norm(insert1(:,j)-I(:,j));
%         ratio_d = aux_delta(i,j)/len;
%         points(46+3*(j-1):48+3*(j-1)) = ratio_d*insert1(:,j) + (1-ratio_d)*I(:,j);
%     end
%     points(37:39) = arrow;
%     trajectory(1:4) = rotm2quat(aux_state(1:3,:,i));
%     trajectory(5:7) = 1.3*arrow;
%     points(40:45) = aux_flag(i,:);
%     points(64:66) = goal_end;
%     points(68:70) = eq_end;
%     if (i == size(aux_state,3)-50)
%         points(67) = 2;
%         pause(0.5);
%     end
%     
%     if (norm(diag(aux_state(7:9,:,i))) > 0.05)
% %        pause(min(0.05,.05/norm(diag(aux_state(7:9,:,i))))); 
%          pause(0.002);
%     end
%         
% %     disp('im running');
%     msg.Data = points;
%     msg1.Orientation.X = trajectory(1);
%     msg1.Orientation.Y = trajectory(2);
%     msg1.Orientation.Z = trajectory(3);
%     msg1.Orientation.W = trajectory(4);
%     msg1.Position.X = trajectory(5);
%     msg1.Position.Y = trajectory(6);
%     msg1.Position.Z = trajectory(7);
%     send(chatpub,msg);
%     points(67) = 0;
%     send(chatpub1,msg1);
% end
%     points(67) = 1;
%     msg.Data = points;
%     send(chatpub,msg);
end