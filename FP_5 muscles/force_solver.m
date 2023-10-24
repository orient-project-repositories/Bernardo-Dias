function [] = force_solver()
thetas = [1.04 0.52 0.52 1.57];
force0 = 0.01*ones(10,1);
fun = @(x)test_points(x,thetas);
fun1 = @(x)force_min(x); %objective function -> requires force and length(which depends on insertion points)
options = optimoptions('fmincon','Display','iter-detailed','MaxFunctionEvaluations',1000000,'MaxIterations',10000,'UseParallel',true,'StepTolerance',1e-12);
force = fmincon(fun1,force0,[],[],[],[],[],[],fun,options);%,fval,exitflag,output,lambda,grad,hessian

max_force = max(force)

