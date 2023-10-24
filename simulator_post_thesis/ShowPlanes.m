function ShowPlanes(sacc,simstatefag,goals,i)
rad_2_deg = 180/pi;
halfrad_2_deg = 2*rad_2_deg;

figindx=10;
figure (figindx+1)
title('XY Plane','FontSize',16);
xlabel('r_x (deg)','FontSize',16);
ylabel('r_y (deg)','FontSize',16);
hold on
line(sacc(1,1:end),sacc(2,1:end));

scatter(sacc(1,:),sacc(2,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);

scatter(sacc(1,end),sacc(2,end),25,'filled','MarkerFaceColor',[0 0 0]);
line([0 0],[-35 35],'Color','b','LineWidth', 1.2);
%         xlim([-0.5 0.5])
%         xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
%         ylim([-0.5 0.5])
xlim([-30 30])
ylim([-30 30])
grid on
% grid minor

figure (figindx+2)
title('XZ Plane','FontSize',16);
xlabel('r_x (deg)','FontSize',16);
ylabel('r_z (deg)','FontSize',16);
hold on
line(sacc(1,1:end-1),sacc(3,1:end-1));

scatter(sacc(1,:),sacc(3,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);

scatter(sacc(1,end),sacc(3,end),25,'filled','MarkerFaceColor',[0 0 0]);
line([0 0],[-35 35],'Color','b','LineWidth', 1.2);
% xlim([-0.4 0.4])
% xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
% ylim([-0.4 0.4])
xlim([-30 30])
ylim([-30 30])
grid on
% grid minor

figure (figindx+3)
title('YZ Plane','FontSize',16);
xlabel('r_y (deg)','FontSize',16);
ylabel('r_z (deg)','FontSize',16);
hold on
line(sacc(2,1:end-1),sacc(3,1:end-1))

scatter(sacc(2,:),sacc(3,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);

scatter(sacc(2,end),sacc(3,end),25,'filled','MarkerFaceColor',[0 0 0]);
plot(goals(2,i)*halfrad_2_deg,goals(3,i)*halfrad_2_deg,'O',  'LineWidth',1.22,...
    'MarkerSize',6,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','green');
% xlim([-0.5 0.5])
% xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
% ylim([-0.5 0.5])
xlim([-30 30])
ylim([-30 30])
grid on
% grid minor

%%  velocity presentation
 if simstatefag==1
     sindx=1;
 else
     sindx=4;
 end

figure(figindx+4)
hold on
l=size(sacc,2)-5;
plot(1:l,abs(sacc(sindx,6:end)*rad_2_deg),'LineWidth',1.2);
grid on
grid minor
xlabel('Time (ms)','fontsize',16);
ylabel('$|\dot{r}_x| (deg/s)$','Interpreter','latex','fontsize',20);
%     legend("eye theta x", "eye theta y", "eye theta z");
%     title('RNN model velocity')
ylim([0 400])

figure(figindx+5)
hold on
plot(1:l,abs(sacc(sindx+1,6:end)*rad_2_deg),'LineWidth',1.2);
grid on
grid minor
% xlabel('time(ms)');
% ylabel('eye velocity y (rad/s)');
xlabel('Time (ms)','fontsize',16);
ylabel('$|\dot{r}_y| (deg/s)$','Interpreter','latex','fontsize',20);
ylim([0 400])

figure(figindx+6)
hold on
plot(1:l,abs(sacc(sindx+2,6:end)*rad_2_deg),'LineWidth',1.2);
grid on
grid minor
% xlabel('time(ms)');
% ylabel('eye velocity z (rad/s)');
xlabel('Time (ms)','fontsize',16);
ylabel('$|\dot{r}_z| (deg/s)$','Interpreter','latex','fontsize',20);
ylim([0 400])

% figure();
% hold on
% plot(1:l+1,sacc(4:end,6:end));
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('eye velocity(rad/s)');
% legend("eye theta x", "eye theta y", "eye theta z");
% title('NARX model velocity')

