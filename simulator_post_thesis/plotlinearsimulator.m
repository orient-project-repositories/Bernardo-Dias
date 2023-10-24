% function plotlinearsimulator()
close all;
clear all; 
clc;
%%all_goals_sequence = [zeros(1,250); 0.3 - 0.5*rand(2,250)]; %For random saccades
initialpos=eye(3);

%For pre-tension testing
% tempgoal = load('pre-tension.mat');
% all_goals_sequence= tempgoal.goal;

%For scaling
% for i = 1:7
%     goal_x(i) = 0;
%     goal_z(i) = (i-1)*0.045;
%     goal_y(i) = 0.06;
% end
% all_goals_sequence = [goal_x; goal_y; goal_z];

%For saccades starting from zero
% tempgoal = load('24sacc.mat');
% all_goals_sequence= tempgoal.all_goals_sacc;

% For 250 random goals
% tempgoal = load('goals3.mat'); 
% all_goals_sequence = tempgoal.tempgoal;

%For continuous saccades
 tempgoal = load('ContinuousTestSet.mat');
 all_goals_sequence= tempgoal.test_set(:,1:24);

%% Initializations
numb = size(all_goals_sequence,2);
XYZ=[];

pt_test = 0 ;
loadsacc_flag = 1; % if load flag is active load saccades and its tau
c_flag =1; % if continuessacc_flag is for continued saccades

extra_time = 200;%100 produced good results
if loadsacc_flag==1 && c_flag == 0
    load('sacc_zeroinit_withtorsion_Bernard_test_set_complete2.mat');
%     load('test_pre-tension.mat');
 %   load('sacc_zeroinit_240_bigger_than_4deg.mat');
end
if loadsacc_flag==1 && c_flag == 1
      load('continuous_saccade_Bernard_test_set_complete2.mat');
     %load('sacc(gridsacc)_Nacc_Eq_ww_Cont_24_70_1.2_0.07_29.4.mat');
end
%% Run and store results
for i=1:numb
    goal_rot_vec=all_goals_sequence(:,i)';
    if i==1
        initial_pos=eye(3);
    else
          initial_pos=eye(3); % for zero initial
%                 quat = rod2quat(initialpos);
%                 initial_pos = quat2rotm(quat);
    end
    
    if loadsacc_flag==1
        statevec=simresult(i).statevec;
        tau_optimal=simresult(i).tau_optimal;
        saccade_ts=simresult(i).saccade_ts;
        P = simresult(i).saccade_ts;
        finalOrientation = statevec(end,1:3);
        acc_err(i) = atan(norm((finalOrientation - goal_rot_vec)))*360/pi;
%         goal_rot_vec=simresult(i).x_des;
    else
        [statevec,tau_optimal,saccade_ts,aux4,P]=runlinearmodel(goal_rot_vec,initial_pos,extra_time,c_flag,pt_test,i);
        simresult(i).statevec=statevec;
        simresult(i).tau_optimal=tau_optimal;
        simresult(i).saccade_ts=saccade_ts-extra_time;
        simresult(i).x_des=goal_rot_vec;
        acc_err(i) = aux4;
    end
    
    simstatefag=0;
    statevec=statevec';
    
    % convert to degree
    sacc_=[];
    sacc_=2*atan(statevec(1:3,:))*180/pi;
    sacc=[sacc_;statevec(4:6,:)];
    CurrGoal = goal_rot_vec*360/pi;
    %It has to receive goal and index of time P
    %ShowPlanesThoroughly(sacc,simstatefag,CurrGoal,P,i);
    if pt_test == 1
        ShowPlanes1(sacc,simstatefag,all_goals_sequence,i); 
    else
        ShowPlanes(sacc,simstatefag,all_goals_sequence,i); %showplanes1 for pre-tension
    end
    XYZ=[XYZ;statevec(1:3,1:end)'];
    
    if c_flag==1
        initialpos=statevec(1:3,end)';
    end
    
end

if loadsacc_flag==0 && c_flag == 0
%    save('test_pre-tension_no average.mat','simresult');
%    save('sacc_zeroinit_240_bigger_than_4deg.mat','simresult');
    save('check_forces_PT=1.mat','simresult');
end

if loadsacc_flag==0 && c_flag == 1
    save('continuous_saccade_Bernard_test_set_complete3.mat','simresult');
end

%% Plotting
XYZd = XYZ*360/pi;
x = XYZ(:,1);
y = XYZ(:,2);
z = XYZ(:,3);

figure,
title('XY Plane');
xlabel('r_x (rad/2)');
ylabel('r_y (rad/2)');
hold on
scatter(x(1:end-1),y(1:end-1),2.5,'filled','MarkerEdgeColor',[0 0 0]);
xlim([-0.5 0.5])
xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
ylim([-0.5 0.5]);
% xlim([-35 35])
% ylim([-35 35])
%xline(0,'b');
grid on
grid minor

% fit a line
p=polyfit(x,y,1);
f = polyval(p,x);
plot(x,f,'-')
% legend('data','linear fit');
% compute the line rotation
t1=atan(p(1))*180/pi;
t2=t1+180;
t3=90-t2;
% Create rotation matrix
theta = t3;
R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
% Rotate the points
point = [x y]';
% rotpoint = R*point;
rotpoint = point;


xr=rotpoint(1,:);
yr=rotpoint(2,:);
figure,
title('XY Plane');
xlabel('r_x (rad/2)');
ylabel('r_y (rad/2)');
hold on
scatter(xr(1:end-1),yr(1:end-1),2.5,'filled','MarkerEdgeColor',[0 0 0]);
xlim([-0.5 0.5])
% xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
ylim([-0.5 0.5]);
% xlim([-35 35])
% ylim([-35 35])
%xline(0,'b');
grid on
grid minor
% fit a new line after the rotation
% p=polyfit(xr,yr,1);
% f = polyval(p,xr);
% plot(xr,f,'-')
% legend('data','linear fit');
figure,
title('XY Plane');
xlabel('r_x (rad/2)');
ylabel('r_y (rad/2)');
hold on
scatter(xr(1:end-1),yr(1:end-1),2.5,'filled','MarkerEdgeColor',[0 0 0]);
xlim([-0.03 0.03])
% xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
ylim([-0.3 0.3]);
%xline(0,'b');
% xlim([-35 35])
% ylim([-35 35])
grid on
grid minor

l=size(xr,2);
zz=zeros(1,l);
for i=1:l
    line([zz(i);xr(i)]',[yr(i);yr(i)]','color','red');
end
%% --------------------------------------------
% if methodnum==1 % eigen vector computation
figure,
plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3),'r.');
hold on;
%compute the normal to the plane and a point that belongs to the plane
%n : a unit (column) vector normal to the plane
%V : a 3 by 2 matrix. The columns of V form an orthonormal basis of the
[n,V,p,evz] = affine_fit(XYZ);
%     fprintf('error (eigen value)=%f \n',evz);
%plot the  point p
plot3(p(1),p(2),p(3),'go','markersize',10,'markerfacecolor','green');
%     v=n/sqrt(sum(n(:).^2))
%     dot(norm(n)
%plot the normal vector
h = quiver3(p(1),p(2),p(3),n(1)/3,n(2)/3,n(3)/3,'b','linewidth',2);

[S1,S2] = meshgrid([-0.4 0 0.4]);
%  [S1,S2] = meshgrid([-40 0 40]);

%generate the point coordinates
X = p(1)+[S1(:) S2(:)]*V(1,:)';
Y = p(2)+[S1(:) S2(:)]*V(2,:)';
Z = p(3)+[S1(:) S2(:)]*V(3,:)';
%plot the plane
surf(reshape(X,3,3),reshape(Y,3,3),reshape(Z,3,3),'facecolor','blue','facealpha',0.5);
[S1,S2] = meshgrid([-0.05 0 0.05]);
%     [S1,S2] = meshgrid([-50 0 5]);
qX = p(1)+[S1(:) S2(:)]*V(1,:)';
qY = p(2)+[S1(:) S2(:)]*V(2,:)';
qZ = p(3)+[S1(:) S2(:)]*V(3,:)';
plot3(qX(2),qY(2),qZ(2),'go','markersize',10,'markerfacecolor','cyan');
P0=[qX(2),qY(2),qZ(2)];
nn=ones(3,size(XYZ,1)).*n;
E1=XYZ-P0;
len=size(XYZ,1);
totalerror=sqrt(sum(abs(dot(E1',nn)).^2)/len);
fprintf('error (using normal vector)=%f \n',totalerror);

xlabel('x');
ylabel('y');
zlabel('z');


%% Plot main sequence properties

peak_velocity = zeros(length(all_goals_sequence),1);
amplitude = zeros(length(all_goals_sequence),1);
duration = zeros(length(all_goals_sequence),1);

for i = 1:length(all_goals_sequence)
    des_orientation = atan(simresult(i).x_des)*360/pi;
    angle(i) = atan2d(des_orientation(2),des_orientation(3));
    max_v_y(i) = max(simresult(i).statevec(:,5)*180/pi);
    max_v_z(i) = max(simresult(i).statevec(:,6)*180/pi);
    
    for j = 1:size(simresult(i).statevec,1)
        norm_velocity(j) = norm(simresult(i).statevec(j,4:6));
    end
    if i == 1 || c_flag == 0
        amplitude(i) = atan(norm(simresult(i).x_des))*360/pi;
    else 
        amplitude(i) = atan(norm(simresult(i).x_des - simresult(i-1).x_des))*360/pi;
    end
    peak_velocity(i) = max(norm_velocity)*180/pi;
%    amplitude(i) = norm(simresult(i).x_des)*360/pi;
    duration(i) = simresult(i).saccade_ts;
end
% % time = linspace(0,max(duration),max(duration));
% myfit = fittype('a + b*log(x)',...
% 'dependent',{'x','y'},'independent',{},...
% 'coefficients',{'a','b'});
figure(50);
hold on
f = fit(amplitude,peak_velocity, 'poly2');
% plot(amplitude,peak_velocity,'o','markerfacecolor',[0 0.75 0.75],'MarkerEdgeColor','k');
scatter(amplitude, peak_velocity,100,angle,'Filled');
plot(f);
xlabel('Amplitude(deg)','fontsize',16);
ylabel('Peak Velocity(deg/s)','fontsize',16);
xlim([0 40])
ylim([0 500])
grid on
title('Peak Velocity vs Amplitude','fontsize',16);
c1 = colorbar;
caxis([min(angle) max(angle)]);
set(get(c1,'Title'),'String','Saccade direction (deg)');

figure(51);
hold on
%  f1 = fit(amplitude,peak_velocity, 'exp2');
 plot(amplitude,peak_velocity,'o','markerfacecolor',[0 0.75 0.75],'MarkerEdgeColor','k');
scatter(amplitude, peak_velocity,100,max_v_y,'Filled');
% plot(f1);
xlabel('Amplitude(deg)','fontsize',16);
ylabel('Peak Velocity(deg/s)','fontsize',16);
xlim([0 40])
ylim([0 500])
grid on
title('Peak Velocity vs Amplitude','fontsize',16);
c1 = colorbar;
caxis([0 400]);
set(get(c1,'Title'),'String','Max Vertical velocity(deg/s)');

figure(52);
hold on
% f = fit(amplitude,peak_velocity, 'exp2');
% plot(amplitude,peak_velocity,'o','markerfacecolor',[0 0.75 0.75],'MarkerEdgeColor','k');
h = scatter(amplitude, peak_velocity,100,max_v_z,'Filled');
% plot(f);
xlabel('Amplitude(deg)','fontsize',16);
ylabel('Peak Velocity(deg/s)','fontsize',16);
xlim([0 40])
ylim([0 500])
grid on
title('Peak Velocity vs Amplitude','fontsize',16);
c1 = colorbar;
caxis([0 400]);
set(get(c1,'Title'),'String','Max Horizontal velocity(deg/s)');

figure(53);
hold on
f2 = fit(amplitude,duration, 'poly1');
plot(amplitude,duration,'o','markerfacecolor',[0 0.75 0.75],'MarkerEdgeColor','k');
plot(f2);
xlabel('Amplitude(deg)','fontsize',16);
ylabel('Duration(ms)','fontsize',16);
xlim([0 40])
ylim([0 200])
grid on
title('Duration vs Amplitude','fontsize',16);

figure(54)
scatter(amplitude,acc_err,'o','markerfacecolor',[0 0.75 0.75],'MarkerEdgeColor','k');
text
xlabel('Amplitude(deg)','fontsize',16);
ylabel('Accuracy (deg)','fontsize',16);
grid on
title('Accuracy vs Amplitude','fontsize',16);
xlim([0 40])
ylim([0 10])
%     xx = linspace(0,saccade_ts,saccade_ts);
%
%     figure();
%     hold ons
%     plot(xx,aux_gaze1);
%     grid on
%     grid minor
%     xlabel('time(ms)');
%     ylabel('eye orentation(rad/s)');
%     legend("eye theta x", "eye theta y", "eye theta z");
%
%     figure();
%     hold on
%     plot(xx,aux_vel_eye);
%     grid on
%     grid minor
%     xlabel('time(ms)');
%     ylabel('eye velocity(rad/s)');
%     legend("eye theta x", "eye theta y", "eye theta z");
