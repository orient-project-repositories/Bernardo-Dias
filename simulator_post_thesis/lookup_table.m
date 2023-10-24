function [theta_eq, eq_rot, rot_err]=lookup_table(goal,theta)

%load('equilibrium_2.5_2.45_change_500000.mat', 'rotvec','theta_used','eq_points');
%load('equilibrium_2_12000.mat', 'rotvec','theta_used','eq_points');
load('equilibrium_points_100k.mat','rotvec','theta_used','eq_points');
%load('equilibrium_2_1.2_change_50000.mat', 'rotvec','theta_used','eq_points');

% % load('equilibrium_2.5_2.45_change_500000_processed.mat', 'rotvec_processed','theta_processed','F_eye_processed');
% % n=10;
% % [B,closest_n]= mink(vecnorm((rotvec_processed-goal)'),n);
% % cut_off=0.015;
% % if any(find(B<cut_off))
% %     %find one with min force within the cut off
% %     t=find(B<cut_off);
% %     combined_force=zeros(size(t,2),1);
% %     for i=1:size(t,2)
% %         combined_force(i)=sum(vecnorm(F_eye_processed(:,:,closest_n(t(i)))));
% %     end
% %     [min_force,smallest]=min(combined_force);
% %     eq_rot = rotvec_processed(closest_n(t(smallest)),:);
% %     rot_err=abs(rotvec_processed(closest_n(t(smallest)),:)-goal);
% %     theta_eq=theta_processed(closest_n(t(smallest)),:);
% % else
% %     %if there are no points within the cut_off, use teh closest
% %     [min_val,closest]=min(vecnorm((rotvec_processed-goal)'));
% %     eq_rot = rotvec_processed(closest,:);
% %     rot_err=abs(rotvec_processed(closest,:)-goal);
% %     theta_eq=theta_processed(closest,:);
% % end
 %% Get 5 closest error values and find closest theta from them
 error = vecnorm((rotvec-goal)');
 indexes = find(error<0.01);
 min_theta_diff = 100;
 if isempty(indexes)
     [min_val,closest]= min(error);
 else
     for i=1:length(indexes)
         if sum((theta-theta_used(indexes(i))).^2) < min_theta_diff
             min_theta_diff = sum((theta-theta_used(indexes(i))).^2);
            closest = indexes(i);
         end
     end
 end

eq_rot = rotvec(closest,:);
rot_err=abs(rotvec(closest,:)-goal);
theta_eq=theta_used(eq_points(closest),:);

end