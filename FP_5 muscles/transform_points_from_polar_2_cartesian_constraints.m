function [I] = transform_points_from_polar_2_cartesian_constraints(insertions)
phi = insertions(1:3);
thetas = insertions(4:8);
z = insertions(9:10);
rs = 0.187689/2; %radius of the sphere
rc = 0.055/2;%radius of the cylinder
rot_angles = [0 0 0 0];

[I_sterno_Laux(1), I_sterno_Laux(2), I_sterno_Laux(3)] = spherical_2_cartesian(thetas(1),phi(1),rs);
[I_sterno_L,~] = head_to_world(I_sterno_Laux,rot_angles); 
I_sterno_R = [I_sterno_L(1) -I_sterno_L(2) I_sterno_L(3)];


[I_sternhyoidaux(1), I_sternhyoidaux(2), I_sternhyoidaux(3)] = spherical_2_cartesian(thetas(2),phi(2),rs);
[I_sternhyoid,~] = head_to_world(I_sternhyoidaux,rot_angles);
I_spine= [-I_sternhyoid(1) I_sternhyoid(2) I_sternhyoid(3)];


[I_trapezius_L1a(1), I_trapezius_L1a(2), I_trapezius_L1a(3)] = spherical_2_cartesian(thetas(3),phi(3),rs);
[I_trapezius_L1,~] = head_to_world(I_trapezius_L1a,rot_angles);
I_trapezius_R1 = [I_trapezius_L1(1) -I_trapezius_L1(2) I_trapezius_L1(3)];


[I_supp_fronta(1), I_supp_fronta(2), I_supp_fronta(3)] = cylindrical_2_cartesian(thetas(4),rc,z(1));
[I_supp_front,~] = neck_to_world(I_supp_fronta,rot_angles);
I_supp_back = [-I_supp_front(1) I_supp_front(2) I_supp_front(3)];


[I_trapezius_L2a(1), I_trapezius_L2a(2), I_trapezius_L2a(3)] = cylindrical_2_cartesian(thetas(5),rc,z(2));
[I_trapezius_L2,~] = neck_to_world(I_trapezius_L2a,rot_angles);
I_trapezius_R2 =[I_trapezius_L2(1) -I_trapezius_L2(2) I_trapezius_L2(3)];


I = [I_sterno_L';I_sterno_R;I_sternhyoid';I_spine;I_trapezius_L1';I_trapezius_R1;I_supp_front';I_supp_back;I_trapezius_L2';I_trapezius_R2];


