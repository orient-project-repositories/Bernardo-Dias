function [c,ceq] = test_points(force,thetas)
th = 0.001;
ceq = [];
points = load('test_points.mat') ;
rotated_points = zeros(10,3);
world_frame_points = zeros(10,3);
rot_zero = [0 0 0 0];
init_length = zeros(10,3);
len_3d = zeros(10,3);
norm_len = zeros(10,1);
force_direction = zeros(10,3);
torque = zeros(10,3);

norm_torque = zeros(10,1);
Q_sterno_L = [0.08 -0.02 0.01];
Q_sterno_R = [0.08 0.02 0.01];
Q_sternhyoid = [0.05 0 0.01];
Q_supp_front = [0.12 0 0.01];
Q_spine = [-0.05 0 0.01];
Q_trapezius_L1 = [-0.07 0.14 0.01];
Q_trapezius_R1 = [-0.07 -0.14 0.01];
Q_supp_back = [-0.12 0 0.01];

Q = [Q_sterno_L;Q_sterno_R;Q_sternhyoid;Q_spine;Q_trapezius_L1;Q_trapezius_R1;Q_supp_front;Q_supp_back;Q_trapezius_L1;Q_trapezius_R1];
%Rotation of the points in world frame
for i = 1:size(points,1)
    if i > 6
        rotated_points(i,:) = world_to_neck(points.I(i,:),thetas);
        world_frame_points(i,:) = neck_to_world(rotated_points(i,:),rot_zero);
    else 
        rotated_points(i,:) = world_to_head(points.I(i,:),thetas);
        world_frame_points(i,:) = head_to_world(rotated_points(i,:),rot_zero);
    end
    init_length(i,:) = Q(i,:) - points.I(i,:);
    len_3d(i,:) = Q(i,:) - world_frame_points(i,:); 
    norm_len(i) = norm(len_3d(i,:));
    force_direction(i,:) = len_3d(i,:)/norm_len(i);
    solver_force = force(i)*force_direction;
    %force(i,:) = (len_3d(i,:)-init_length(i,:))*force_direction(i,:)';
    
%   norm_force(i) = norm(force(i,:));
    torque(i,:) = cross(solver_force(i,:),rotated_points(i,:));% I can instead use the force jacobian to compute joint torque 
    [tau,tau_g] = build_jacobian(points.I,thetas,force,size(points.I,1),force_direction');
    norm_torque(i) = norm(torque(i,:));
end

%max_force = max(norm_force);
%max_torque = max(norm_torque)
c(1:10) = -force ;
c(11) = sum(tau(1,:)) + sum(tau_g(1,1)) - th
c(12) = sum(tau(2,:)) + sum(tau_g(2,1)) - th
c(13) = sum(tau(3,:)) + sum(tau_g(3,1)) - th
c(14) = sum(tau(4,:)) + sum(tau_g(4,1)) - th
