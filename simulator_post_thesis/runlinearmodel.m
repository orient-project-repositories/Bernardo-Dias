function [statevec,tau_optimal,saccade_ts,aux4,P,tan_points,insertions]=runlinearmodel(goal_rot_vec,R_eye,extra_time,c_flag,pt_test,i)
dimension='prototype';


Ts = 0.001; % timestep of 1 ms
 
k = 20; % spring constant 20
 
% initial motor positions
if c_flag == 1
    c = 2;
    t = minimization(R_eye,k,c);
    theta = t';
elseif pt_test == 1
%theta = [1.50238469494650;1.66073697629457;1.49761596835154;1.33926369318071;1.47853230638836;1.52146834767573]';
if i==1 || i== 5 || i == 9 || i == 13
%     c = 1;
    theta = [1 1 1 1 1 1];
end

if i==2 || i== 6 || i == 10 || i == 14
%     c = 2;
    theta = 2*[1 1 1 1 1 1];
end

if i==3 || i== 7 || i == 11 || i == 15
%     c = 3;
    theta = 3*[1 1 1 1 1 1];
end

if i==4 || i== 8 || i == 12 || i == 16
%     c = 4;
    theta = 4*[1 1 1 1 1 1];
end
%  t = minimization(eye(3),k,c);
%  theta = t';
else
        theta = [     1.0191
    1.0000
    1.0000
    1.0403
    1.0000
    1.0289]';
%     theta = [     2.0189
%     1.9630
%     1.9811
%     2.0370
%     1.9715
%     2.0285]';
%Best
%theta = [1.0016 1.1072 0.9984 0.8928 0.9857 1.0143];
end
%paper thetas for WA
% theta = [     1.0012
%     1.0555
%     0.9988
%     0.8334
%     0.9857
%     1.0143]' ;



%    
%    theta = [    
%     3.0048
%     3.3215
%     2.9952
%     2.6785
%     2.9571
%     3.0429
% ]';      
%    
% %     final?
% theta = [     1.1999
%     1.0497
%     1.2001
%     0.7503
%     1.1829
%     1.2171]';   
   
% theta = [2.01893712481317;1.96299387485996;1.98106292418320;2.03700617391867;1.97146423659847;2.02853581254045]';



% R_eye = eye(3); % eye starting orientation in world frame
omega_eye = zeros(3,1); % eye angular velocity(x y z)

R_head = eye(3); % head starting orientation in world frame
omega_head = zeros(3,1); % head angular velocity(x y z)

%%

[~,~,~,tau_optimal,aux_gaze1, ~,~,~,aux_vel_eye,~,saccade_ts,aux4,P]=run_optimal_control(goal_rot_vec,R_eye,R_head,omega_eye,omega_head,theta,Ts,k,dimension,extra_time);

statevec=[aux_gaze1,aux_vel_eye];
% tau_optimal=aux_tau_eye;

