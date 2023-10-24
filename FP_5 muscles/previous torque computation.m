%Previous torque computation
%function[complete_torque,g_offset,len,force] = get_torque(insertions,force,thetas,numb)
% rotated_points = zeros(10,3);
% all_rotated_points = zeros(numb*2*size(insertions,1),3);
% o_h = [0 ;-0.263; 0]; % in world frame


l_rotated = zeros(1,10);
g_head = zeros(1,numb);
g_neck1 = zeros(1,numb);
g_neck2 = zeros(1,numb);
g_neck3 = zeros(1,numb);
% v_aux1 = [0.5428    0.5767   -0.6106]; %sterno_L
% %v2 = [0.5428   -0.5767   -0.6106]; %sterno_R
% v_aux2 = [0.9145   -0.1769   -0.3638]; %supp_front

%Define insertion points on the torso in world frame
Q_sterno_L = [0.08 -0.02 0.01];
Q_sterno_R = [0.08 0.02 0.01];
Q_sternhyoid = [0.05 0 0.01];
Q_supp_front = [0.12 0 0.01];
Q_spine = [-0.05 0 0.01];
Q_trapezius_L1 = [-0.07 0.14 0.01];
Q_trapezius_R1 = [-0.07 -0.14 0.01];
Q_supp_back = [-0.12 0 0.01];

Q = [Q_sterno_L;Q_sterno_R;Q_sternhyoid;Q_spine;Q_trapezius_L1;Q_trapezius_R1;Q_supp_front;Q_supp_back;Q_trapezius_L1;Q_trapezius_R1];
len = zeros(numb,size(Q,1));

I = transform_points_from_polar_2_cartesian(insertions);

%Compute rotations of the insertion points for 'numb' rotations
 for i=1:numb
    %neck_rotation = rotz(thetas(i,4)*pi/180)*roty(thetas(i,3)*pi/180)*rotx(thetas(i,2)*pi/180);%rotationVectorToMatrix([thetas(i,2) thetas(i,3) thetas(i,4)]); %[rx ry rz] in radians
%     neck_rotation_aux = [thetas(i,2) thetas(i,3) thetas(i,4)];%rotz(thetas(4)*pi/180)*roty(thetas(3)*pi/180)*rotx(thetas(2)*pi/180);%rotationVectorToMatrix([thetas(i,2) thetas(i,3) thetas(i,4)]); %[rx ry rz] in radians
%     neck_rotation = eul2rotm(neck_rotation_aux);
    %neck_rotation = quat2rotm(eul2quat(neck_rotation_aux,'XYZ'));
    %Rotate insertion points on the head according to previously defined rotations
%     for k = 1:size(I,1) %everything in world frame
%         if k > 6
%             a = I(k,:)';
%             b = neck_rotation*a;
%             rotated_points(k,:) = b'; 
%         end
% 
%           rotated_point1 = I(k,:)'; 
%           rotated = rotz(thetas(i,1)*pi/180)*rotated_point1;
% 
%           rotated2 = head_to_neck(rotated,[0 0 0 0]);
%           rotated3_aux = neck_rotation*rotated2;
% %         rotated3 = world_to_neck(rotated3_aux',[0 0 0 0]);
% %         rotated_points(k,:) = neck_to_head(rotated3,[0 0 0 0]);
%          
% 
%     end
    
   %all_rotated_points(i*size(I,1)-(size(I,1)-1):i*size(I,1),:) = rotated_points; %all rotated points
      for j = 1:size(I,1)%size(all_rotated_points,1)
        if j > 6
            [Q_rotated(j,:),~] = world_to_neck(Q(j,:),thetas(i,:));
            len_3d(j,:) = Q_rotated(j,:)-I(j,:);
        else
            [Q_rotated(j,:),~] = world_to_head(Q(j,:),thetas(i,:));
            len_3d(j,:) = Q_rotated(j,:)-I(j,:);
        end 
           l_rotated(j) = norm(len_3d(j,:));%l_rotated(i,j)
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

