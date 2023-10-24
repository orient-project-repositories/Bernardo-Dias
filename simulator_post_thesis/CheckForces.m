function [eqRot,final_forces] = CheckForces(goal,PT,WA,LT,grid,tau)

D_eye = 2*[0.02 0 0; 0 0.02 0; 0 0 0.02];
dimension = 'prototype';
goal_orientation = goal;
eqRot = [];
final_forces = [];
final_torque = [];
k = 1;
th_rx = 0.01;   %threshold for lookup table proximity torsional orientation values
th_rz = 0.01;   %threshold for lookup table proximity horizontal orientation values
th_theta = 0.5; %threshold for lookup table proximity theta values
if grid == 1
    if WA == 1
        aux = load('equilibrium_points_100k_WA.mat','rotvec','theta_used','eq_points');
        error = vecnorm((aux.rotvec-goal)');
        indx = find(error<0.05);
        min_theta_diff = 100;
        if isempty(indx)
            [min_val,closest]= min(error);
        else
            for i=1:length(indx)
                if sum((PT-aux.theta_used(aux.eq_points(indx(i)))).^2) < min_theta_diff
                    min_theta_diff = sum((PT-aux.theta_used(aux.eq_points(indx(i)))).^2);
                    closest = indx(i);
                end
            end
            eqRot = aux.rotvec(closest,:)*180/pi;
            final_theta = aux.theta_used(aux.eq_points(closest),:);
            [f_norm,tau_k, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~,~,~,~] = compute_taus_WA(rotationVectorToMatrix(eqRot), 0,final_theta,20,0,D_eye,dimension);
            final_forces = f_norm;
            final_torque = tau_k;
        end
        %     error = vecnorm(([aux.rotvec(:,1) aux.rotvec(:,3)] - goal_orientation)'); %vector with error from goal orientation for all equilibrium orientations stored
        %     indx = find(error <th_orientation); % List of indexes corresponding to equilibrium orientations close to goal orientation
    else
        aux = load('equilibrium_points_100k.mat','rotvec','theta_used','eq_points');
        error = vecnorm((aux.rotvec-goal)');
        indx = find(error<0.05);
        min_theta_diff = 100;
        if isempty(indx)
            [min_val,closest]= min(error);
        else
            for i=1:length(indx)
                if sum((PT-aux.theta_used(aux.eq_points(indx(i)))).^2) < min_theta_diff
                    min_theta_diff = sum((PT-aux.theta_used(aux.eq_points(indx(i)))).^2);
                    closest = indx(i);
                end
            end
            eqRot = aux.rotvec(closest,:)*180/pi;
            final_theta = aux.theta_used(aux.eq_points(closest),:);
            [~, ~, f_norm, f, ~,~,tau_k] = compute_eye_torques2(rotationVectorToMatrix(eqRot), 0,final_theta,20,0,D_eye,dimension);
            final_forces = f_norm;
            final_torque = tau_k;
        end
    end
    
end
if WA==1 && LT == 1 && grid == 0
    aux = load('equilibrium_points_100k_WA.mat','rotvec','theta_used','eq_points');
    error = [aux.rotvec(:,1) aux.rotvec(:,3)] - [goal_orientation(1) goal_orientation(3)];
    indx = find(error(:,1)<th_rx & error(:,2)<th_rz);
    %     error = vecnorm(([aux.rotvec(:,1) aux.rotvec(:,3)] - goal_orientation)'); %vector with error from goal orientation for all equilibrium orientations stored
    %     indx = find(error <th_orientation); % List of indexes corresponding to equilibrium orientations close to goal orientation
    for i = 1:length(indx)
        if sum((PT-aux.theta_used(aux.eq_points(indx(i)))).^2) < th_theta
            eqRot(k,:) = aux.rotvec(indx(i),:)*180/pi;
            final_theta(k,:) = aux.theta_used(aux.eq_points(indx(i)),:);
            [f_norm,tau_k, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~,~,~,~] = compute_taus_WA(rotationVectorToMatrix(eqRot(k,:)), 0,final_theta(k,:),20,0,D_eye,dimension);
            final_forces = [final_forces;f_norm];
            final_torque = [final_torque;tau_k];
            k = k+1;
        end
    end
    
elseif WA==0 && LT == 1 && grid == 0
    aux = load('equilibrium_points_100k.mat','rotvec','theta_used','eq_points');
    %     error = vecnorm(([aux.rotvec(:,1) aux.rotvec(:,3)] - goal_orientation)'); %vector with error from goal orientation for all equilibrium orientations stored
    error = [aux.rotvec(:,1) aux.rotvec(:,3)] -[goal_orientation(1) goal_orientation(3)];
    %     indx = find(error <th_orientation); % List of indexes corresponding to equilibrium orientations closer to goal orientation
    indx = find(error(:,1)<th_rx & error(:,2)<th_rz);
    for i = 1:length(indx)
        if sum((PT-aux.theta_used(aux.eq_points(indx(i)))).^2) < th_theta
            eqRot(k,:) = aux.rotvec(indx(i),:)*180/pi;
            final_theta(k,:) = aux.theta_used(aux.eq_points(indx(i)),:);
            [~, ~, f_norm, f, ~,~,tau_k] = compute_eye_torques2(rotationVectorToMatrix(eqRot(k,:)), 0,final_theta(k,:),20,0,D_eye,dimension);
            final_forces = [final_forces;f_norm];
            final_torque = [final_torque;tau_k];
            k = k+1;
        end
    end
elseif WA == 0 && LT == 0 && grid == 0
    aux = load('sacc_zeroinit_240_bigger_than_4deg.mat');
    %aux = load('sacc_zeroinit_withtorsion_Bernard_test_set_complete2.mat');
    %aux = load('check_forces_PT=1.mat');
    %indx = find(abs(aux.rotvec(:,1))<0.01);
    for i = 1:length(aux.simresult)
        eqRot(i,:) = aux.simresult(i).x_des*360/pi;
        final_theta(i,:) = aux.simresult(i).tau_optimal(:,end)' + PT;
        [~, ~, f_norm, f, ~,~,tau_k] = compute_eye_torques2(rotationVectorToMatrix(eqRot(i,:)), 0,final_theta(i,:),20,0,D_eye,dimension);
        final_forces = [final_forces;f_norm];
        final_torque = [final_torque;tau_k];
    end
elseif WA == 1 && LT == 0 && grid == 0
    %aux = load('sacc_zeroinit_withtorsion_Bernard_test_set_complete_WA.mat');
    aux = load('sacc_zeroinit_WA_PT=2.mat');
    %     indx = find(abs(aux.rotvec(:,1))<0.01);
    for i = 1:length(aux.simresult)
        eqRot(i,:) = aux.simresult(i).x_des*360/pi;
        final_theta(i,:) = aux.simresult(i).tau_optimal(:,end)' + PT;
        [f_norm,tau_k, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~,~,~,~] = compute_taus_WA(rotationVectorToMatrix(eqRot(i,:)), 0,final_theta(k,:),20,0,D_eye,dimension);
        final_forces = [final_forces;f_norm];
        final_torque = [final_torque;tau_k];
    end
end
if isempty(eqRot)
    return;
end
% For plotting surfaces
[X,Y] = meshgrid(eqRot(:,2),eqRot(:,3));
Z1 = repmat(final_forces(:,1),1,length(eqRot));
Z2 = repmat(final_forces(:,2),1,length(eqRot));
Z3 = repmat(final_forces(:,3),1,length(eqRot));
Z4 = repmat(final_forces(:,4),1,length(eqRot));
Z5 = repmat(final_forces(:,5),1,length(eqRot));
Z6 = repmat(final_forces(:,6),1,length(eqRot));


%% Plotting all muscles' forces

% figure(51)
% hold on
% scatter3(eqRot(:,2),eqRot(:,3),final_theta(:,2),[],final_theta(:,1));
% xlabel('rotation angle y(rad)');
% ylabel('rotation angle z');
% zlabel('theta');
% title('IR theta');
% cb1=colorbar;
% set(get(cb1,'Title'),'String','theta (rad)')

figure(52)
hold on
scatter3(eqRot(:,2),eqRot(:,3),final_theta(:,2),[],final_theta(:,2));
xlabel('rotation angle y(rad)');
ylabel('rotation angle z');
zlabel('theta');
title('MR theta');
cb1=colorbar;
set(get(cb1,'Title'),'String','theta (rad)')

% figure(53)
% hold on
% scatter3(eqRot(:,2),eqRot(:,3),final_theta(:,2),[],final_theta(:,3));
% xlabel('rotation angle y(rad)');
% ylabel('rotation angle z');
% zlabel('theta');
% title('SR theta');
% cb1=colorbar;
% set(get(cb1,'Title'),'String','theta (rad)')
% 
% figure(54)
% hold on
% scatter3(eqRot(:,2),eqRot(:,3),final_theta(:,2),[],final_theta(:,4));
% xlabel('rotation angle y(rad)');
% ylabel('rotation angle z');
% zlabel('theta');
% title('LR theta');
% cb1=colorbar;
% set(get(cb1,'Title'),'String','theta (rad)')
% 
% figure(55)
% hold on
% scatter3(eqRot(:,2),eqRot(:,3),final_theta(:,2),[],final_theta(:,5));
% xlabel('rotation angle y(rad)');
% ylabel('rotation angle z');
% zlabel('theta');
% title('IO theta');
% cb1=colorbar;
% set(get(cb1,'Title'),'String','theta (rad)')
% 
% figure(56)
% hold on
% scatter3(eqRot(:,2),eqRot(:,3),final_theta(:,2),[],final_theta(:,6));
% xlabel('rotation angle y(rad)');
% ylabel('rotation angle z');
% zlabel('theta');
% title('SO theta');
% cb1=colorbar;
set(get(cb1,'Title'),'String','theta (rad)')

if tau == 1
%     figure(1);
%     hold on
%     % surf(X,Y,Z1);
%     scatter3(eqRot(:,2),eqRot(:,3),final_torque(:,2),[],final_torque(:,1));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('elastic torque_{mesured}');
%     title('IR torque');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','torque (N)')

    figure(2);
    hold on
    % surf(X,Y,Z2);
    scatter3(eqRot(:,2),eqRot(:,3),final_torque(:,2),[],final_torque(:,2));
    xlabel('rotation angle y(rad)');
    ylabel('rotation angle z');
    zlabel('elastic torque_{mesured}');
    title('MR torque');
    cb1=colorbar;
    set(get(cb1,'Title'),'String','torque (N)')
    
%     figure(3);
%     hold on
%     % surf(X,Y,Z3);
%     scatter3(eqRot(:,2),eqRot(:,3),final_torque(:,2),[],final_torque(:,3));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('elastic torque_{mesured}');
%     title('SR torque');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','torque (N)')
%     
%     figure(4);
%     hold on
%     % surf(X,Y,Z4);
%     scatter3(eqRot(:,2),eqRot(:,3),final_torque(:,4),[],final_torque(:,4));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('elastic torque_{mesured}');
%     title('LR torque');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','torque (N)')
%     
%     figure(5);
%     hold on
%     % surf(X,Y,Z5);
%     scatter3(eqRot(:,2),eqRot(:,3),final_torque(:,2),[],final_torque(:,5));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('elastic torque_{mesured}');
%     title('IO torque');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','torque (N)')
%     %
%     figure(6);
%     hold on
%     % surf(X,Y,Z6);
%     scatter3(eqRot(:,2),eqRot(:,3),final_torque(:,2),[],final_torque(:,6));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('elastic torque_{mesured}');
%     title('SO torque');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','torque (N)')
else
%     figure(1);
%     hold on
%     % surf(X,Y,Z1);
%     scatter3(eqRot(:,2),eqRot(:,3),final_forces(:,2),[],final_forces(:,1));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('F_{mesured}');
%     title('IR forces');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','Force (N)')

    figure(2);
    hold on
    % surf(X,Y,Z2);
    scatter3(eqRot(:,2),eqRot(:,3),final_forces(:,2),[],final_forces(:,2));
    xlabel('rotation angle y(rad)');
    ylabel('rotation angle z');
    zlabel('F_{mesured}');
    title('MR forces');
    cb1=colorbar;
    set(get(cb1,'Title'),'String','Force (N)')
    
%     figure(3);
%     hold on
%     % surf(X,Y,Z3);
%     scatter3(eqRot(:,2),eqRot(:,3),final_forces(:,2),[],final_forces(:,3));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('F_{mesured}');
%     title('SR forces');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','Force (N)')
%     
%     figure(4);
%     hold on
%     % surf(X,Y,Z4);
%     scatter3(eqRot(:,2),eqRot(:,3),final_forces(:,4),[],final_forces(:,4));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('F_{mesured}');
%     title('LR forces');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','Force (N)')
%     
%     figure(5);
%     hold on
%     % surf(X,Y,Z5);
%     scatter3(eqRot(:,2),eqRot(:,3),final_forces(:,2),[],final_forces(:,5));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('F_{mesured}');
%     title('IO forces');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','Force (N)')
%  
%     figure(6);
%     hold on
%     % surf(X,Y,Z6);
%     scatter3(eqRot(:,2),eqRot(:,3),final_forces(:,2),[],final_forces(:,6));
%     xlabel('rotation angle y(rad)');
%     ylabel('rotation angle z');
%     zlabel('F_{mesured}');
%     title('SO forces');
%     cb1=colorbar;
%     set(get(cb1,'Title'),'String','Force (N)')
end
save('StaticForces.mat','eqRot','final_forces');
    % if WA == 0 && LT == 0
    %
    %     line( [eqRot(1,2) eqRot(3,2)], [eqRot(1,3) eqRot(3,3)], [final_forces(1,2) final_forces(3,2)],'color','r');
    %     line( [eqRot(1,2) eqRot(6,2)], [eqRot(1,3) eqRot(6,3)], [final_forces(1,2) final_forces(6,2)],'color','r' );
    %
    %     line( [eqRot(4,2) eqRot(7,2)], [eqRot(4,3) eqRot(7,3)], [final_forces(4,2) final_forces(7,2)],'color','r' );
    %     line( [eqRot(4,2) eqRot(8,2)], [eqRot(4,3) eqRot(8,3)], [final_forces(4,2) final_forces(8,2)],'color','r');
    %
    %     line( [eqRot(9,2) eqRot(17,2)], [eqRot(9,3) eqRot(17,3)], [final_forces(9,2) final_forces(17,2)],'color','r' );
    %     line( [eqRot(9,2) eqRot(21,2)], [eqRot(9,3) eqRot(21,3)], [final_forces(9,2) final_forces(21,2)],'color','r');
    %
    %     line( [eqRot(11,2) eqRot(19,2)], [eqRot(11,3) eqRot(19,3)], [final_forces(11,2) final_forces(19,2)],'color','r' );
    %     line( [eqRot(11,2) eqRot(23,2)], [eqRot(11,3) eqRot(23,3)], [final_forces(11,2) final_forces(23,2)],'color','r');
    %
    %     line( [eqRot(10,2) eqRot(18,2)], [eqRot(10,3) eqRot(18,3)], [final_forces(10,2) final_forces(18,2)],'color','r' );
    %     line( [eqRot(10,2) eqRot(22,2)], [eqRot(10,3) eqRot(22,3)], [final_forces(10,2) final_forces(22,2)],'color','r');
    %
    %     line( [eqRot(12,2) eqRot(20,2)], [eqRot(12,3) eqRot(20,3)], [final_forces(12,2) final_forces(20,2)],'color','r');
    %     line( [eqRot(12,2) eqRot(24,2)], [eqRot(12,3) eqRot(24,3)], [final_forces(12,2) final_forces(24,2)],'color','r' );
    % end