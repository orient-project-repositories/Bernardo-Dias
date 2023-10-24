function [I] = transform_points_from_polar_2_cartesian(insertions)
phi = insertions(1:3);
thetas = insertions(4:8);
z = insertions(9:10);
rs = 0.187689/2; %radius of the sphere
rc = 0.055/2;%radius of the cylinder
rot_angles = [0 0 0 0];

[I_sterno_L(1), I_sterno_L(2), I_sterno_L(3)] = spherical_2_cartesian(thetas(1),phi(1),rs);
[aux,~] = head_to_world(I_sterno_L,rot_angles); 
rot= world_to_head([aux(1) -aux(2) aux(3)],rot_angles);
I_sterno_R = rot';

[I_sternhyoid(1), I_sternhyoid(2), I_sternhyoid(3)] = spherical_2_cartesian(thetas(2),phi(2),rs);
[aux1,~] = head_to_world(I_sternhyoid,rot_angles);
rot1 = world_to_head([-aux1(1) aux1(2) aux1(3)],rot_angles);
I_spine = rot1';

[I_trapezius_L1(1), I_trapezius_L1(2), I_trapezius_L1(3)] = spherical_2_cartesian(thetas(3),phi(3),rs);
[aux3,~] = head_to_world(I_trapezius_L1,rot_angles);
rot3 = world_to_head([aux3(1) -aux3(2) aux3(3)],rot_angles);
I_trapezius_R1 = rot3';

[I_supp_front(1), I_supp_front(2), I_supp_front(3)] = cylindrical_2_cartesian(thetas(4),rc,z(1));
[aux2,~] = neck_to_world(I_supp_front,rot_angles);
rot2 = world_to_neck([-aux2(1) aux2(2) aux2(3)],rot_angles);
I_supp_back = rot2';

[I_trapezius_L2(1), I_trapezius_L2(2), I_trapezius_L2(3)] = cylindrical_2_cartesian(thetas(5),rc,z(2));
[aux4,~] = neck_to_world(I_trapezius_L2,rot_angles);
rot4 = world_to_neck([aux4(1) -aux4(2) aux4(3)],rot_angles);
I_trapezius_R2 = rot4';

I = [I_sterno_L;I_sterno_R;I_sternhyoid;I_spine;I_trapezius_L1;I_trapezius_R1;I_supp_front;I_supp_back;I_trapezius_L2;I_trapezius_R2];


