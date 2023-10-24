function[complete_torque,g_offset,len,force] = get_torque(insertions,force,thetas,numb)

l_rotated = zeros(1,10);
g_head = zeros(1,numb);
g_neck1 = zeros(1,numb);
g_neck2 = zeros(1,numb);
g_neck3 = zeros(1,numb);

%Define insertion points on the torso in world frame
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

Q = [Q_sterno_L;Q_sterno_R;Q_sternhyoid;Q_spine;Q_trapezius_L1;Q_trapezius_R1;Q_supp_front;Q_supp_back;Q_trapezius_L2;Q_trapezius_R2];
len = zeros(numb,size(Q,1));

I = transform_points_from_polar_2_cartesian(insertions);

%Compute rotations of the insertion points for 'numb' rotations
 for i=1:numb
     Q_rotated = zeros(10,3);
     len_3d = zeros(10,3);
     v_aux = zeros(10,3);
      for j = 1:size(I,1)
        if j > 6
            Q_rotated(j,:) = world_to_neck(Q(j,:),thetas(i,:));
            len_3d(j,:) = Q_rotated(j,:)-I(j,:);
        else
            Q_rotated(j,:) = world_to_head(Q(j,:),thetas(i,:));
            len_3d(j,:) = Q_rotated(j,:)-I(j,:);
        end 
           l_rotated(j) = norm(len_3d(j,:));
           v_aux(j,:)= len_3d(j,:)/l_rotated(j);
      end

     v_= v_aux';
     v(3*i-2:3*i,:) = v_;

    len(i,:) = l_rotated;
    
    [torque,g_offset_] = build_jacobian(I,thetas(i,:),force(i*size(I,1)-(size(I,1)-1):i*size(I,1),1),size(I,1),v_);
    complete_torque(4*i-3:4*i,:) = torque;
    g_head(i) = g_offset_(1);
    g_neck1(i) = g_offset_(2);
    g_neck2(i) = g_offset_(3);
    g_neck3(i) = g_offset_(4);
    
 end
g_offset = [g_head;g_neck1;g_neck2;g_neck3];
