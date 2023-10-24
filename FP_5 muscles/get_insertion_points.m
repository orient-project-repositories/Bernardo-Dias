function [solution,max_force] = get_insertion_points()
tic 
numb = 20;
thetas = zeros(numb,4);
% insertions0 = zeros(10+10*numb,1);
insertions0 = zeros(5,3);
I_sterno_L = [0; -0.0808445;0.21]';
I_sterno_R = [0;0.0808445;0.21]';% 
I_sternhyoid = [0.05;0;0.19]';
I_spine = [-0.05;0;0.19]';
I_supp_front = [0.025;0;0.16]';
I_trapezius_L1 = [-0.07;0.04;0.21]';
I_trapezius_R1 = [-0.07;-0.04;0.21]';%
I_supp_back = [-0.025;0;0.16]'; 
I_trapezius_L2 = [0.025;0.022;0.035]';%
I_trapezius_R2 = [0.025;-0.022;0.035]';%
I0 =[I_sterno_L;I_sterno_R;I_sternhyoid;I_spine;I_trapezius_L1;I_trapezius_R1;I_supp_front;I_supp_back;I_trapezius_L2;I_trapezius_R2];
k = 0;
for i=1:2:length(I0)  
    k = k+1;
    if i> 6
        insertions0(k,:) = cartesian_2_cylindrical(I0(i,:)); 
    else
        insertions0(k,:) = cartesian_2_spherical(I0(i,:)-[0 0 0.20694]); 
    end
end

%Arrange insertions0 so that it's a colums vector plus adding force
init_insert = [insertions0(1:3,2);insertions0(:,1);insertions0(4:5,3);zeros(10*numb,1)];



%Generate random orientations in radians
for i=1:numb
    thetas(i,1) = -1.04 + 2.08*rand; %120 degrees (+-60) %head uniform distribution
    thetas(i,2:3) = -0.52 + 1.04*rand(1,2); %60 degrees (+-30) %torsion and vertical neck movements
    thetas(i,4) = -1.57 + 3.14*rand; %180 degrees (+-90) % horizontal neck movement
end

%Sampling points on the most eccentric orientations possible
% for i = 1:numb
%     thetas(i,1) = 2.08 -0.08*rand; %120 degrees (+-60) %head uniform distribution
%     thetas(i,2:3) = 1.04 - 0.04*rand; %60 degrees (+-30) %torsion and vertical neck movements
%     thetas(i,4) = 3.14 - 0.14*rand; %180 degrees (+-90) % horizontal neck movement
% end

% %Define constraints and cost function for fmincon solver
fun = @(x)constraints(x(1:10),x(11:end),thetas,numb);
fun1 = @(x)minimization(x(11:end),x(1:10),numb,thetas); %objective function -> requires force and length(which depends on insertion points)
options=optimoptions('fmincon','Display','iter-detailed','MaxFunctionEvaluations',numb*1000000,'MaxIterations',200000,'StepTolerance',1e-15,'TolFun',4e-5);%,'UseParallel',true
solution = fmincon(fun1,init_insert,[],[],[],[],[],[],fun,options);%,fval,exitflag,output,lambda,grad,hessian
 
%Divide solution in insertion points and force
force = solution(11:end);
max_force = max(force)

%Visualize insertion points
setup_simscape(solution);
 
%save optimized points
assignin('base','polar_coord',solution(1:10));
assignin('base','force',force);
assignin('base','rotation_angles',thetas);

elapsed_time = toc