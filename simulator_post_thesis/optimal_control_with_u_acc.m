function [u_optimal_final,saccade_duration,P] = optimal_control_with_u_acc(ss,goal,x_initial,bound_theta,if_antagonist_muscle,theta,dimension)

%use the u_diff and just put a delay after the optimization

Tc = 0.001;

C_vel = zeros(3,12);
C_vel(1:3,7:9) = eye(3);

% u_agonist_elements = zeros(3,9);
% u_agonist_elements(1,1) = 1*if_antagonist_muscle(1);
% u_agonist_elements(1,3) = -1*if_antagonist_muscle(1);
% u_agonist_elements(2,2) = 1*if_antagonist_muscle(2);
% u_agonist_elements(2,4) = -1*if_antagonist_muscle(2);
% u_agonist_elements(3,5) = 1*if_antagonist_muscle(3);
% u_agonist_elements(3,6) = -1*if_antagonist_muscle(3);

%y_des = [goal;zeros(3,1)];
y_des = goal;
y_vel_des= zeros(3,1);
%y_des = [goal;zeros(3,1)];
min_duration = 0.001;%0.03; %needed?
max_duration = 1;
nr_iterations = 150;%250;%150

if strcmp(dimension,'prototype') == 1
    
 lambda_1 = 1*10^-3;%1*10^-2; % position weight
lambda_2 = 2*10^-3;%1*10^-1; % energy coefficient
lambda_3 = 3*10^-3;%8*10^-3; % time weight
lambda_4 = 100; % equilibrium weight 1*10^2
lambda_5 = 0*10^-7; % last command accuracy weight
lambda_6 = 0*10^-3; % velocity weight   
    
    
    
    %for saccades starting from rest
%             lambda_1 = 1*10^-3;%1*10^-2; % position weight
%             lambda_2 = 8*10^-4;%1*10^-1; % energy coefficient
%             lambda_3 = 1*10^-3;%8*10^-3; % time weight
%             lambda_4 = 100; % equilibrium weight 1*10^2
%             lambda_5 = 0*10^-5; % last command accuracy weight
%             lambda_6 = 0*10^-2; % velocity weight
    
% attempt for continuous saccades
%             lambda_1 = 1*10^-3;%1*10^-2; % position weight
%             lambda_2 = 8*10^-4;%1*10^-1; % energy coefficient
%             lambda_3 = 1*10^-3;%8*10^-3; % time weight
%             lambda_4 = 100; % equilibrium weight 1*10^2
%             lambda_5 = 2*10^-6; % last command accuracy weight
%             lambda_6 = 0*10^-2; % velocity weight
    
%     if norm(goal)> 0.3 %% FOR THE 4 SACCADES TEST SET
%             lambda_1 = 6*10^-5;%1*10^-2; % position weight
%             lambda_2 = 1*10^-4;%1*10^-1; % energy coefficient
%             lambda_3 = 6*10^-2; % time weight
%             lambda_4 = 0*10^2; % equilibrium weight 1*10^2
%             lambda_5 = 1*10^-15; % last command accuracy weight
%             lambda_6 = 1*10^-5; % velocity weight
%     else
%             lambda_1 = 1*10^-4;%1*10^-2; % position weight
%             lambda_2 = 1*10^-3;%1*10^-1; % energy coefficient
%             lambda_3 = 9*10^-4;%8*10^-3; % time weight
%             lambda_4 = 0*10^2; % equilibrium weight 1*10^2
%             lambda_5 = 1*10^-12; % last command accuracy weight
%             lambda_6 = 1*10^-5; % velocity weight
%      end
%     if norm(goal)> 0.3
%             lambda_1 = 6*10^-5;%1*10^-2; % position weight
%             lambda_2 = 1*10^-4;%1*10^-1; % energy coefficient
%             lambda_3 = 6*10^-2; % time weight
%             lambda_4 = 0*10^2; % equilibrium weight 1*10^2
%             lambda_5 = 1*10^-15; % last command accuracy weight
%             lambda_6 = 1*10^-5; % velocity weight
%     else
%             lambda_1 = 1*10^-4;%1*10^-2; % position weight
%             lambda_2 = 1*10^-5;%1*10^-1; % energy coefficient
%             lambda_3 = 1*10^-3;%8*10^-3; % time weight
%             lambda_4 = 0*10^2; % equilibrium weight 1*10^2
%             lambda_5 = 1*10^-12; % last command accuracy weight
%             lambda_6 = 1*10^-5; % velocity weight
%     end

    
    %FOR MAIN SEQUENCE
    %         lambda_1 = 1*10^-1; % position weight
    %         lambda_2 = 5*10^-4; % energy coefficient
    %         lambda_3 = 2*10^-3; % time weight
    %         lambda_4 = 1*10^2; % equilibrium weight
    %         lambda_5 = 1*10^-7; % last command accuracy weight
    %         lambda_6 = 0*10^-3; % velocity weight
    
%             lambda_1 = 7*10^-3;%1*10^-2; % position weight
%             lambda_2 = 8*10^-3;%1*10^-1; % energy coefficient
%             lambda_3 = 1*10^-2;%8*10^-3; % time weight
%             lambda_4 = 100; % equilibrium weight 1*10^2
%             lambda_5 = 0*10^-5; % last command accuracy weight
%             lambda_6 = 0*10^-2; % velocity weight
%     lambda_1 = 1*10^-20;%1*10^-2; % position weight
%     lambda_2 = 1*10^-4;%1*10^-1; % energy coefficient
%     lambda_3 = 2*10^-20;%8*10^-3; % time weight
%     lambda_4 = 0; % equilibrium weight 1*10^2
%     lambda_5 = 1*10^-14; % last command accuracy weight
%     lambda_6 = 2*10^-1; % velocity weight




%Current 
%     if norm(goal)> 0.3
%             lambda_1 = 6*10^-3;%1*10^-2; % position weight
%             lambda_2 = 6*10^-2;%1*10^-1; % energy coefficient
%             lambda_3 = 8*10^0;%8*10^-3; % time weight
%             lambda_4 = 100; % equilibrium weight 1*10^2
%             lambda_5 = 0*10^-12; % last command accuracy weight
%             lambda_6 = 9*10^-3; % velocity weight
%     else
%             lambda_1 = 1*10^-2;%1*10^-2; % position weight
%             lambda_2 = 9*10^-3;%1*10^-1; % energy coefficient
%             lambda_3 = 1*10^-1;%8*10^-3; % time weight
%             lambda_4 = 100; % equilibrium weight 1*10^2
%             lambda_5 = 0*10^-12; % last command accuracy weight
%             lambda_6 = 10*10^-4; % velocity weight
%     end    


%Best so far for bigger saccades
%             lambda_1 = 1*10^-3;%1*10^-2; % position weight
%             lambda_2 = 9*10^-3;%1*10^-1; % energy coefficient
%             lambda_3 = 2*10^-1;%8*10^-3; % time weight
%             lambda_4 = 100; % equilibrium weight 1*10^2
%             lambda_5 = 1*10^-7; % last command accuracy weight
%             lambda_6 = 10*10^-3; % velocity weight
    % time discount factor
    beta = 1/30;
elseif strcmp(dimension,'real') == 1
    lambda_1 = 1*10^-1; % position weight
    lambda_2 = 5*10^-4; % energy coefficient
    lambda_3 = 9*10^-4; % time weight
    lambda_4 = 1*10^2; % equilibrium weight
    lambda_5 = 1*10^-7; % last command accuracy weight
    lambda_6 = 0*10^-3; % velocity weight
    % time discount factor
    beta = 1/30;
end





extra_iter = min_duration/Tc; % only used to plot cost function

cost_function = zeros(nr_iterations,1);
cost_function_1 = zeros(nr_iterations,1);
cost_function_3 = zeros(nr_iterations,1);
cost_function_2 = zeros(nr_iterations,1);
cost_function_4 = zeros(nr_iterations,1);
cost_function_5 = zeros(nr_iterations,1);
cost_function_6 = zeros(nr_iterations,1);

A = ss.A;
B = ss.B;
C = ss.C;

if (max_duration-min_duration)/Tc < nr_iterations
    nr_iterations = round((max_duration-min_duration)/Tc);
end
max_timesteps= round((min_duration + nr_iterations * Tc)/Tc)+1; %basically 130
u_optimal = zeros(9*max_timesteps,nr_iterations);

for p = 0:nr_iterations - 1
    nr_timesteps = round((min_duration + p * Tc)/Tc)+1;

    for i = 1:nr_timesteps 
        F_gama = zeros(12,9*(nr_timesteps));
    
    for j = 0:nr_timesteps-2
        if j<=i-1
            F_gama(1:12,9*j+1:9*j+9) = A^(i-1-j)*B;
        else
            F_gama(1:12,9*j+1:9*j+9) = zeros(12,9);
        end
    end
    
    F_gama(1:12,9*(nr_timesteps-1)+1:9*nr_timesteps) = zeros(12,9);
    L = A^i;
    G_row = C*F_gama;
    F_row = C*L;
    G_row_vel = C_vel*F_gama;
    F_row_vel = C_vel*L;

    end

nrg_coef = zeros(9*(nr_timesteps));
for k = 1:9
    nrg_coef(k,k) = 1;
end
for k= 9+1 : 9*nr_timesteps
    nrg_coef(k,k-9) = -1;
    nrg_coef(k,k) = 1;
end

store_last_u = zeros(9,9*nr_timesteps);
store_last_u(1:9,9*(nr_timesteps-1)+1:9*nr_timesteps) = eye(9);


e = -(eye(12)-A)*L*x_initial; %not F_row
E = -(eye(12)-A)*F_gama+B*store_last_u; %not G_row

%optimizing u using quadprog -> min 1/2 u'Hu + f'u and 


last_u = zeros(9*(nr_timesteps));
last_u(end-8:end-3,end-8:end-3) = eye(6);
command_term_squared = lambda_5 * last_u;

%assuming x_initial to be zero

accuracy_term_squared=0;
accuracy_term_linear=0;

for i=0:19%nr_iterations/10 -1
    y_pred_acc_aux = C*(A^i*(F_gama + B*store_last_u) + B*store_last_u);
    aux_squared = ((y_pred_acc_aux)'*(y_pred_acc_aux))';
    accuracy_term_squared = accuracy_term_squared+aux_squared;
    aux_linear = (-2*goal'*C*(A^i*(F_gama + B*store_last_u)+B*store_last_u) )';
    accuracy_term_linear =  accuracy_term_linear + aux_linear; 

end

%position_term_squared = lambda_1 * (G_row'*G_row);
position_term_squared = lambda_1 * accuracy_term_squared;
velocity_term_squared = lambda_6 * (G_row_vel'*G_row_vel);


energy_term_squared = lambda_2*(nrg_coef'*nrg_coef);
equilibrium_term_squared = lambda_4*(E'*E);


u_last_des = zeros(9*(nr_timesteps),1);
u_last_des(end-8:end-3)= bound_theta';
command_term_linear = -lambda_5*2*u_last_des; %minus?

%assuming x_initial to be zero
%position_term_linear = lambda_1*2*G_row'*(F_row*x_initial-y_des);
position_term_linear = lambda_1*accuracy_term_linear;
velocity_term_linear = lambda_6*2*G_row_vel'*(F_row_vel*x_initial-y_vel_des);


equilibrium_term_linear = lambda_4*2*E'*e;

H = 2*(command_term_squared + position_term_squared + energy_term_squared + equilibrium_term_squared+velocity_term_squared);
f = command_term_linear +position_term_linear + equilibrium_term_linear+ velocity_term_linear;


options = optimoptions('quadprog','Display','off');

u_optimal(1:9*(nr_timesteps), p + 1) = quadprog(H,f,[],[],[],[],[],[],[],options);
acc_cost = 0;

for i=0:19%nr_iterations/10 -1
   aux_acc = C*(A^i*(F_gama + B*store_last_u) + B*store_last_u);
   summation_acc = norm(aux_acc*u_optimal(1:9*(nr_timesteps), p + 1) - y_des);
   acc_cost = acc_cost+ summation_acc; 
end

%y_pred = G_row*u_optimal(1:9*(nr_timesteps),p + 1)+ F_row*x_initial;
y_vel_pred = G_row_vel*u_optimal(1:9*(nr_timesteps),p + 1)+ F_row_vel*x_initial;

cost_function_1(p + 1) = lambda_1*acc_cost^2; %accuracy cost
% cost_function_1(p + 1) = lambda_1*norm(b - y_des)^2; %accuracy cost
cost_function_6(p + 1) = lambda_6*norm(y_vel_pred - y_vel_des)^2; %velocity accuracy cost
%cost_function_6(p + 1) = lambda_6*(vel_cost)^2; %velocity accuracy cost
cost_function_5(p + 1) = lambda_5*norm(u_optimal(9*(nr_timesteps-1)+1:9*(nr_timesteps-1)+6,p+1)-bound_theta)^2;
cost_function_2(p + 1) = lambda_2*u_optimal(1:9*(nr_timesteps),p + 1)'*nrg_coef'*nrg_coef*u_optimal(1:9*(nr_timesteps),p + 1);
cost_function_3(p + 1) = lambda_3*(1-1/(1 + beta*p));
cost_function_4(p + 1) = u_optimal(1:9*(nr_timesteps),p+1)'*equilibrium_term_squared*u_optimal(1:9*(nr_timesteps),p+1) + equilibrium_term_linear'*u_optimal(1:9*(nr_timesteps),p+1) + lambda_4*(e'*e);
cost_function(p + 1) = cost_function_1(p + 1) + cost_function_2(p + 1) + cost_function_4(p + 1) + cost_function_3(p + 1)+cost_function_5(p + 1) + cost_function_6(p + 1); %


end
    

delay = 0;
[s,P] = min(cost_function);
saccade_duration = min_duration+(P-1)*Tc;
nr_ts_of_solution = round((min_duration + (P-1) * Tc)/Tc);
u_optimal_9xnr_ts = reshape(u_optimal(1:9*nr_ts_of_solution,P),[9,nr_ts_of_solution]);
u_optimal_final = [u_optimal_9xnr_ts, repmat(u_optimal_9xnr_ts(:,end),1,delay)];

% x = 1:size(cost_function,1) + extra_iter;
% figure()
% % plotting costs
% plot(x(extra_iter:end-1),cost_function_1,'-b','LineWidth',1.2);
% xlabel('Time(ms)','FontSize',18);
% ylabel('Cost function','FontSize',18);
% hold on
% plot(x(extra_iter:end-1),cost_function_2,'g','LineWidth',1.5);
% plot(x(extra_iter:end-1),cost_function_3,'MarkerFaceColor',[0.9290 0.6940 0.1250],'LineWidth',1.5);
% %plot(x(extra_iter:end-1),cost_function_4,'k','LineWidth',1.5);%equilibrium
% % plot(x(extra_iter:end-1),cost_function_5,'c','LineWidth',1.5);
% % plot(x(extra_iter:end-1),cost_function_6,'MarkerFaceColor',[0.3290 0.6940 0.5250]); % velocity
% plot(x(extra_iter:end-1),cost_function,'MarkerFaceColor',[0.4660 0.6740 0.1880],'LineWidth',1.5);
% plot(P+extra_iter,s,'o','MarkerEdgeColor','k');
% xlim([0 extra_iter+nr_iterations]);
% legend('accuracy on position','energy term','duration term ','total cost');
% %
% 
% % % figure()
% % % hold on
% % % for i=1:6
% % % %plotting u_optimal to see its shape
% % % plot(u_optimal_tot(i,:));
% % % end
% %
% figure()
% 
% % % plot(u_optimal_9xnr_ts(1,:));
% % % plot(u_optimal_9xnr_ts(3,:));
% subplot(1,3,1);
% hold on
% plot(theta(1)+u_optimal_9xnr_ts(1,:),'LineWidth',1.2);
% plot(theta(3)+u_optimal_9xnr_ts(3,:),'LineWidth',1.2);
% legend('m1','m3');
% % % legend('IR','SR', 'IR+t', 'SR+t');
% axis([0 150 0 3]);
% xlabel('Time','FontSize',18);
% ylabel('motor command (\tau)','FontSize',18);
% % % %
% % % figure()
% % % hold on
% % % % plot(u_optimal_9xnr_ts(2,:));
% % % % plot(u_optimal_9xnr_ts(4,:));
% subplot(1,3,2);
% hold on
% plot(theta(2)+u_optimal_9xnr_ts(2,:),'LineWidth',1.2);
% plot(theta(4)+u_optimal_9xnr_ts(4,:),'LineWidth',1.2);
% legend('m2','m4')
% axis([0 150 0 3]);
% xlabel('Time','FontSize',18);
% ylabel('motor command (\tau)','FontSize',18);
% % % % legend('MR','LR', 'MR+t', 'LR+t');
% % %
% % % figure()
% % % hold on
% % % % plot(u_optimal_9xnr_ts(5,:));
% % % % plot(u_optimal_9xnr_ts(6,:));
% subplot(1,3,3);
% hold on
% plot(theta(5)+u_optimal_9xnr_ts(5,:),'LineWidth',1.2);
% plot(theta(6)+u_optimal_9xnr_ts(6,:),'LineWidth',1.2);
% legend('m5','m6');
% axis([0 150 0 3]);
% xlabel('Time','FontSize',18);
% ylabel('motor command (\tau)','FontSize',18);
% % % % legend('IO','SO', 'IO+t', 'SO+t');

end