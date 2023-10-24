function simulator_get_equ_points()
clear all
close all

tic
%ticBytes(gcp);

Ts = 0.001;
omega_eye = 0*[5;2;1]; % x y z
k = 20;
R_eye = eye(3);


%theta =  [3.0284    2.9445   2.9716    3.0555  2.9572    3.0428];
%theta = [2.01893712481317;1.96299387485996;1.98106292418320;2.03700617391867;1.97146423659847;2.02853581254045]';
% theta=[2.52367139983403;2.45374233758514;2.47632864916171;2.54625771120175;2.46433028969333;2.53566975944555]';


numb =100000;
theta=repmat([2.01893712481317;1.96299387485996;1.98106292418320;2.03700617391867;1.97146423659847;2.02853581254045]',numb,1);
rotations=zeros(3,3,numb);
fail_flag=zeros(numb,1);
total_timesteps=zeros(numb,1);

fprintf('Progress:\n');
fprintf(['\n' repmat('.',1,numb) '\n\n']);
% 
parfor i=1:numb
    theta(i,:)=theta(i,:)-1.5+2*1.5*rand(1,6);
    [R_eq,total_time,fail] = check_equilibrium(R_eye,omega_eye,Ts,theta(i,:),k);
    rotations(:,:,i)= R_eq;
    fail_flag(i)=fail;
    total_timesteps(i) = total_time;
    
    fprintf('\b|\n');
end

assignin('base', 'R_equilibrium',rotations);
assignin('base', 'total_timesteps',total_timesteps);
assignin('base', 'fail_flag',fail_flag);
assignin('base', 'theta_used',theta);

%tocBytes(gcp)
toc
end
