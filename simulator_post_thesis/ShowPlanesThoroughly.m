function ShowPlanes(sacc,simstatefag,goal,p,SaccadeNr) 
figindx=400;
if SaccadeNr < 17 
    if round(sacc(2,end))~= 0
    figure (figindx+1)
    title('XY Plane');
    xlabel('r_x (deg)');
    ylabel('r_y (deg)');
    hold on
    line(sacc(1,1:end),sacc(2,1:end));
    scatter(sacc(1,:),sacc(2,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);
    scatter(sacc(1,end),sacc(2,end),25,'filled','MarkerFaceColor',[0 0 0]);
    scatter(goal(1),goal(2),25,'filled','MarkerFaceColor',[0 0 1]);
    scatter(sacc(1,p),sacc(2,p),25,'filled','MarkerFaceColor',[0 1 0]);
    if rem(SaccadeNr,2)==0

            text(sacc(1,p)-1.5,sacc(2,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)-1.5,goal(2),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)-1.5,sacc(2,end),int2str(SaccadeNr),'FontSize',6);
    else

            text(sacc(1,p)+1.5,sacc(2,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)+1.5,goal(2),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)+1.5,sacc(2,end),int2str(SaccadeNr),'FontSize',6);
  
        
    end
    %         xlim([-0.5 0.5])
    %         xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
    %         ylim([-0.5 0.5])
    xlim([-35 35])
    ylim([-35 35])
    grid on
    grid minor
    legend('','','final orientation','goal orientation','orientation at time p');
    end
    if round(sacc(3,end))~= 0
    figure (figindx+2)
    title('XZ Plane');
    xlabel('r_x (deg)');
    ylabel('r_z (deg)');
    hold on
    line(sacc(1,1:end-1),sacc(3,1:end-1));
    % scatter(sacc(1,1:end-1),sacc(3,1:end-1),2.5,'filled','MarkerEdgeColor',[0 0 0]);
    scatter(sacc(1,:),sacc(3,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);
    scatter(sacc(1,end),sacc(3,end),25,'filled','MarkerFaceColor',[0 0 0]);
    scatter(goal(1),goal(3),25,'filled','MarkerFaceColor',[0 0 1]);
    scatter(sacc(1,p),sacc(3,p),25,'filled','MarkerFaceColor',[0 1 0]);
    if rem(SaccadeNr,2)==0

            text(sacc(1,p)-1.5,sacc(3,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)-1.5,goal(3),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)-1.5,sacc(3,end),int2str(SaccadeNr),'FontSize',6);

    else

            text(sacc(1,p)+1.5,sacc(3,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)+1.5,goal(3),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)+1.5,sacc(3,end),int2str(SaccadeNr),'FontSize',6);

        
    end
    
    % xlim([-0.4 0.4])
    % xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
    % ylim([-0.4 0.4])
    xlim([-35 35])
    ylim([-35 35])
    grid on
    grid minor
    legend('','','final orientation','goal orientation','orientation at time p');
    end

else
    figure (500)
    title('XY Plane');
    xlabel('r_x (deg)');
    ylabel('r_y (deg)');
    hold on
    line(sacc(1,1:end),sacc(2,1:end));
    scatter(sacc(1,:),sacc(2,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);
    scatter(sacc(1,end),sacc(2,end),25,'filled','MarkerFaceColor',[0 0 0]);
    scatter(goal(1),goal(2),25,'filled','MarkerFaceColor',[0 0 1]);
    scatter(sacc(1,p),sacc(2,p),25,'filled','MarkerFaceColor',[0 1 0]);
    if rem(SaccadeNr,2)==0

            text(sacc(1,p)-1.5,sacc(2,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)-1.5,goal(2),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)-1.5,sacc(2,end),int2str(SaccadeNr),'FontSize',6);

    else

            text(sacc(1,p)+1.5,sacc(2,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)+1.5,goal(2),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)+1.5,sacc(2,end),int2str(SaccadeNr),'FontSize',6);

        
    end
    %         xlim([-0.5 0.5])
    %         xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
    %         ylim([-0.5 0.5])
    xlim([-35 35])
    ylim([-35 35])
    grid on
    grid minor
    legend('','','final orientation','goal orientation','orientation at time p');
    
    figure (501)
    title('XZ Plane');
    xlabel('r_x (deg)');
    ylabel('r_z (deg)');
    hold on
    line(sacc(1,1:end-1),sacc(3,1:end-1));
    % scatter(sacc(1,1:end-1),sacc(3,1:end-1),2.5,'filled','MarkerEdgeColor',[0 0 0]);
    scatter(sacc(1,:),sacc(3,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);
    scatter(sacc(1,end),sacc(3,end),25,'filled','MarkerFaceColor',[0 0 0]);
    scatter(goal(1),goal(3),25,'filled','MarkerFaceColor',[0 0 1]);
    scatter(sacc(1,p),sacc(3,p),25,'filled','MarkerFaceColor',[0 1 0]);
    if rem(SaccadeNr,2)==0

            text(sacc(1,p)-1.5,sacc(3,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)-1.5,goal(3),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)-1.5,sacc(3,end),int2str(SaccadeNr),'FontSize',6);
  
    else

            text(sacc(1,p)+1.5,sacc(3,p),int2str(SaccadeNr),'FontSize',6);
            text(goal(1)+1.5,goal(3),int2str(SaccadeNr),'FontSize',6);
            text(sacc(1,end)+1.5,sacc(3,end),int2str(SaccadeNr),'FontSize',6);
  
        
    end
    
    % xlim([-0.4 0.4])
    % xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
    % ylim([-0.4 0.4])
    xlim([-35 35])
    ylim([-35 35])
    grid on
    grid minor
    legend('','','final orientation','goal orientation','orientation at time p');
   
end
figure (figindx+3)
title('YZ Plane');
xlabel('r_y (deg)');
ylabel('r_z (deg)');
hold on
line(sacc(2,1:end-1),sacc(3,1:end-1))
scatter(sacc(2,:),sacc(3,:),0.5,'filled','MarkerEdgeColor',[1 0 0]);
scatter(sacc(2,end),sacc(3,end),25,'filled','MarkerFaceColor',[0 0 0]);
scatter(goal(2),goal(3),25,'filled','MarkerFaceColor',[0 0 1]);
scatter(sacc(2,p),sacc(3,p),25,'filled','MarkerFaceColor',[0 1 0]);
if(rem(SaccadeNr,2)==0 && SaccadeNr<=8)
    text(sacc(2,p)+1,sacc(3,p),int2str(SaccadeNr));
    text(goal(2)+1,goal(3),int2str(SaccadeNr));
    text(sacc(2,end)+1,sacc(3,end),int2str(SaccadeNr));
end
if(rem(SaccadeNr,2)~=0 && SaccadeNr<=8)
    text(sacc(2,p)-1.5,sacc(3,p),int2str(SaccadeNr));
    text(goal(2)-1.5,goal(3),int2str(SaccadeNr));
    text(sacc(2,end)-1.5,sacc(3,end),int2str(SaccadeNr));
end
if(rem(SaccadeNr,2)==0 && SaccadeNr>8 && SaccadeNr <= 16)
    text(sacc(2,p),sacc(3,p)+1,int2str(SaccadeNr));
    text(goal(2),goal(3)+1,int2str(SaccadeNr));
    text(sacc(2,end),sacc(3,end)+1,int2str(SaccadeNr));
end
if(rem(SaccadeNr,2)~=0 && SaccadeNr>8 && SaccadeNr <= 16)
    text(sacc(2,p),sacc(3,p)-1.5,int2str(SaccadeNr));
    text(goal(2),goal(3)-1.5,int2str(SaccadeNr));
    text(sacc(2,end),sacc(3,end)-1.5,int2str(SaccadeNr));
end
if SaccadeNr > 16
    text(sacc(2,p)+1.5,sacc(3,p),int2str(SaccadeNr));
    text(goal(2)+1.5,goal(3),int2str(SaccadeNr));
    text(sacc(2,end)+1.5,sacc(3,end),int2str(SaccadeNr));
end
% xlim([-0.5 0.5])
% xticks([-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5])
% ylim([-0.5 0.5])
xlim([-35 35])
ylim([-35 35])
grid on
grid minor
legend('','','final orientation','goal orientation','orientation at time p');
%%  velocity presentation
 if simstatefag==1
     sindx=1;
 else
     sindx=4;
 end
figure(figindx+4)
hold on
l=size(sacc,2)-5;
plot(1:l,abs(sacc(sindx,6:end)),'LineWidth',1.2);
grid on
grid minor
xlabel('Time','fontsize',16);
ylabel('$\dot{r}_x (rad/s)$','Interpreter','latex','fontsize',20);
%     legend("eye theta x", "eye theta y", "eye theta z");
%     title('RNN model velocity')
ylim([-10 10])

figure(figindx+5)
hold on
plot(1:l,abs(sacc(sindx+1,6:end)),'LineWidth',1.2);
grid on
grid minor
% xlabel('time(ms)');
% ylabel('eye velocity y (rad/s)');
xlabel('Time','fontsize',16);
ylabel('$\dot{r}_y (rad/s)$','Interpreter','latex','fontsize',20);
ylim([-10 10])

figure(figindx+6)
hold on
plot(1:l,abs(sacc(sindx+2,6:end)),'LineWidth',1.2);
grid on
grid minor
% xlabel('time(ms)');
% ylabel('eye velocity z (rad/s)');
xlabel('Time','fontsize',16);
ylabel('$\dot{r}_z (rad/s)$','Interpreter','latex','fontsize',20);
ylim([-10 10])

% figure();
% hold on
% plot(1:l+1,sacc(4:end,6:end));
% grid on
% grid minor
% xlabel('time(ms)');
% ylabel('eye velocity(rad/s)');
% legend("eye theta x", "eye theta y", "eye theta z");
% title('NARX model velocity')

