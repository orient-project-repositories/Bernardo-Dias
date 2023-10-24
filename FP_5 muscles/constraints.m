function [c,ceq] = constraints(insertions,force,thetas,numb)

%Inicializations
Q_sterno_L = [0.08 -0.02 0.01];
Q_sterno_R = [0.08 0.02 0.01];
Q_sternhyoid = [0.05 0 0.01];
Q_supp_front = [0.12 0 0.01];
Q_spine = [-0.05 0 0.01];
Q_trapezius_L1 = [-0.07 0.14 0.01];
Q_trapezius_R1 = [-0.07 -0.14 0.01];
Q_trapezius_L2 = [-0.08 0.12 0.01];
Q_trapezius_R2 = [-0.08 -0.12 0.01];
Q_supp_back = [-0.12 0 0.01];
direction1 = [0.357406317813216,0.271827608802327,-0.893515794533039];
direction5 = [0,0.447213595499958,-0.894427190999916];
Q = [Q_sterno_L;Q_sterno_R;Q_sternhyoid;Q_spine;Q_trapezius_L1;Q_trapezius_R1;Q_supp_front;Q_supp_back;Q_trapezius_L2;Q_trapezius_R2];

ceq = zeros(1,1);
th = 0.001; 
phi = insertions(1:3);
theta = insertions(4:8);
z = insertions(9:10);

[complete_torque,g_offset,~] = get_torque(insertions,force,thetas,numb);
I = transform_points_from_polar_2_cartesian_constraints(insertions);
direction = get_direction(Q,I);
% min_dist = get_min_distance(Q,I);

% c = zeros(4*numb + 20 + size(force,1) + length(min_dist),1);
c = zeros(4*numb + 20 + size(force,1),1);
for j=1:numb
c(4*j-3) = sum(complete_torque(4*j-3,:)) + sum(g_offset(1,j)) - th;
c(4*j-2) = sum(complete_torque(4*j-2,:)) + sum(g_offset(2,j)) - th;
c(4*j-1) = sum(complete_torque(4*j-1,:)) + sum(g_offset(3,j)) - th;
c(4*j) = sum(complete_torque(4*j,:)) + sum(g_offset(4,j)) - th;
end

c(4*j+1:4*j+2) = -z(1:2) + 0.04;
c(4*j+3:4*j+4) = z(1:2) - 0.168;
c(4*j+5:4*j+7) = -phi;
c(4*j+8:4*j+10) = phi - pi;
c(4*j+11:4*j+13) = -theta(1:3) + pi/6;
c(4*j+14:4*j+16) = theta(1:3) - (11/6)*pi;
c(4*j+17:4*j+18) = -theta(4:5);
c(4*j+19:4*j+20) = theta(4:5) - 2*pi;
c(4*j+21:4*j+21+size(force,1)-1) = -force;
% c(4*j+21+size(force,1))=- dot(direction(1,:),direction1) + 0.95;
% c(4*j+21+size(force,1)+1)=- dot(direction(5,:),direction5) + 0.95;
% c(4*j+21+size(force,1):4*j+21+size(force,1)+length(min_dist)-1) = -min_dist + 0.01;


