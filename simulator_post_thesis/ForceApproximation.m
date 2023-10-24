%% Force Approximation
% F = x'*Q*x + Q2*x so that we have a squared term and linear terms, in a way to better approximate force

X = zeros(size(rotation_vector,1),9);%9);
X(:,1) = rotation_vector(:,1).^2;
X(:,2) = rotation_vector(:,2).^2;
X(:,3) = rotation_vector(:,3).^2;
X(:,4) = rotation_vector(:,1).*rotation_vector(:,2);
X(:,5) = rotation_vector(:,1).*rotation_vector(:,3);
X(:,6) = rotation_vector(:,2).*rotation_vector(:,3);
X(:,7) = rotation_vector(:,1);
X(:,8) = rotation_vector(:,2);
X(:,9) = rotation_vector(:,3);

%Least Squares Method
f1 = abs(vecnorm(F1_all') -  vecnorm(F3_all'));
f2 = abs(vecnorm(F2_all') -  vecnorm(F4_all'));
f3 = abs(vecnorm(F5_all') -  vecnorm(F6_all'));
f = (f1+f2+f3)';

coef = (X'*X)\X'*f; % coefficients that relate ori3entation with outputed force
Q = [coef(1) coef(4)/2 coef(5)/2;coef(4)/2 coef(2) coef(6)/2; coef(5)/2 coef(6)/2 coef(3)];
Q2 = [coef(7) ; coef(8); coef(9)];
f_ = zeros(size(rotation_vector,1),1);
for i=1:size(rotation_vector,1)
   f_(i) = rotation_vector(i,:)*Q*rotation_vector(i,:)' + Q2'*rotation_vector(i,:)';%+ff*r(i,:)'; 
end

figure();
scatter3(rotation_vector(:,1),rotation_vector(:,2),f,[],f);
xlabel('rotation angle x(rad)');
ylabel('rotation angle y');
zlabel('F_{mesured}');
title('Sum of differential force in xy plane (measured)');
cb1=colorbar;
caxis([0 4])
set(get(cb1,'Title'),'String','Force (N)')

figure();
scatter3(rotation_vector(:,1),rotation_vector(:,2),f_,[],f_);
xlabel('rotation angle x(rad)');
ylabel('rotation angle y');
zlabel('F_{approximated}');
title('Sum of differential force in xy plane (approximated)');
cb2=colorbar;
caxis([0 4])
set(get(cb2,'Title'),'String','Force (N)')

figure();
scatter3(rotation_vector(:,1),rotation_vector(:,3),f,[],f);
xlabel('rotation angle x(rad)');
ylabel('rotation angle z');
zlabel('F_{mesured}');
title('Sum of differential force in xz plane (measured)');
cb3=colorbar;
caxis([0 4])
set(get(cb3,'Title'),'String','Force (N)')

figure();
scatter3(rotation_vector(:,1),rotation_vector(:,3),f_,[],f_);
xlabel('rotation angle x(rad)');
ylabel('rotation angle z');
zlabel('F_{approximated}');
title('Sum of differential force in xz plane (approximated)');
cb4=colorbar;
caxis([0 4])
set(get(cb4,'Title'),'String','Force (N)')

figure();
scatter3(rotation_vector(:,2),rotation_vector(:,3),f,[],f);
xlabel('rotation angle y(rad)');
ylabel('rotation angle z');
zlabel('F_{mesured}');
title('Sum of differential force in yz plane (measured)');
cb5=colorbar;
caxis([0 4])
set(get(cb5,'Title'),'String','Force (N)')

figure();
scatter3(rotation_vector(:,2),rotation_vector(:,3),f_,[],f_);
xlabel('rotation angle y(rad)');
ylabel('rotation angle z');
zlabel('F_{approximated}');
title('Sum of differential force in yz plane (approximated)');
cb6=colorbar;
caxis([0 4])
set(get(cb6,'Title'),'String','Force (N)')
