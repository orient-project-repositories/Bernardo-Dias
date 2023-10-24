eq_points=find(fail_flag<=0);
rotvec=zeros(size(eq_points,1),3);
point=[1;0;0];
points=zeros(3,size(eq_points,1));
for i=1:size(eq_points,1)
    temp_quat=rotm2quat(R_equilibrium(:,:,eq_points(i)));
    rotvec(i,:) = quat2rod(temp_quat);
    points(:,i) = R_equilibrium(:,:,eq_points(i))*point;
end

figure()
title('equilibrium points in rotation vectors');
scatter3(rotvec(:,1),rotvec(:,2),rotvec(:,3));
xlabel('rx');
ylabel('ry');
zlabel('rz');

figure()
title('equilibrium points shown in cartesian coordinates');
scatter3(points(1,:),points(2,:),points(3,:));
xlabel('x');
ylabel('y');
zlabel('z');