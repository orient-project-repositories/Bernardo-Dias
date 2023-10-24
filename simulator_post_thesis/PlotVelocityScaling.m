%% Check component scaling 
load('sacc_zeroinit_check_compnent_scaling.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Reza, you have to run this with the nonlinear control and simulator
%For scaling
% for i = 1:7
%     goal_x(i) = 0;
%     goal_z(i) = (i-1)*0.045;
%     goal_y(i) = 0.06;
% end
% 
% all_goals_sequence = [goal_x; goal_y; goal_z];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
amp = [0 5 10 15 20 25 30];
color = [[1 0 0]; [0 1 0] ; [0 0 1]; [1 1 0]; [0 1 1] ; [1 0 1] ;[0 0 0]];
for i=1:length(simresult)
  angle(i) = atan2d(simresult(i).x_des(3),simresult(i).x_des(2));
  velocity_v = simresult(i).statevec(:,5)*180/pi;
  velocity_h = simresult(i).statevec(:,6)*180/pi;
  max_v(i) = max(velocity_v); 
  a(i) = cos(angle(i))*max_v(1);
  figure(1);
  hold on
  plot(-velocity_h,'color',color(i,:));
  plot(velocity_v,'color',color(i,:));
  xlabel('time(ms)');
  ylabel('component velocity (deg/s)','fontsize',16);
  title('Component Scaling','fontsize',16);
  figure(2)
  hold on
  scatter(angle(i),max(velocity_v));
  xlabel('direction angle(deg)');
  ylabel('max vertical velocity(deg/s)','fontsize',16);
  title('Maximum velocity decay','fontsize',16);
  velocity_v = 0;
  velocity_h = 0;
end
figure(1)
legend('','Vz(0)','','Vz(5)','','Vz(10)','','Vz(15)','','Vz(20)','','Vz(25)','','Vz(30)');
