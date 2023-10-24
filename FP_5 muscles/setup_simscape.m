function [] = setup_simscape(solution)

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
neck_goal = eye(3);
head_goal = 0;
assignin('base','neck_goal',neck_goal);
assignin('base','head_goal',head_goal);
thetas = [0 0 0 0];

%% For old points
%I = solution;
%% For new points
points = transform_points_from_polar_2_cartesian(solution);

%For visualization, put the points in world frame!!!!!! (points from solver are in body frame)
for i = 1:10
    if i > 6
      [I(i,:),~] = neck_to_world(points(i,:),thetas);
    else
        [I(i,:),~] = head_to_world(points(i,:),thetas);
    end
end


%I = [I_sterno_L;I_sterno_R;I_sternhyoid;I_spine;I_trapezius_L1;I_trapezius_R1;I_supp_front;I_supp_back;I_trapezius_L2;I_trapezius_R2];

assignin('base','I',I);
assignin('base','points_body_frame',points);
assignin('base','Q',Q);

%set_param('head_neck_model_no_links_5muscles','SimMechanicsOpenEditorOnUpdate','on');
design(Q,I);
sim('head_neck_model_no_links_5muscles',0.001)
