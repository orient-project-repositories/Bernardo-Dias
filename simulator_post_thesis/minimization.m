function [thetas,c,ceq] = minimization(R,k,c)
fun = @(x)get_theta1(x,R,k);
fun1 = @cost;

theta0 = zeros(6,1);
%average > 2
A = -[0.5 0 0.5 0 0 0; 0 0.5 0 0.5 0 0; 0 0 0 0 0.5 0.5];
b = -c*[1 1 1];
% A=-eye(6);
% b=-c*ones(1,6);
options=optimoptions('fmincon','Display','iter-detailed');
thetas = fmincon(fun1,theta0,A,b,[],[],[],[],fun,options);
[c,ceq]=fun(thetas);
