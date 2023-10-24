function [A,B] = get_jacobian(S,Q,R_eye_world,R_head_world,I_eye,I_head,omega_eye,omega_head,theta,k,dimension)

%inicializations
eta_eye = zeros(3,1);
r = 0.024;
eta_head = zeros(3,1);
k_neck = 35*[1 0 0; 0 1 0; 0 0 1];
d = [0.04,0.04,0.04,0.04,0.065,0.065];

l1_0 = norm(S(:,1)-Q(:,1)) + d(1);%0.151502735392456;
l2_0 = norm(S(:,2)-Q(:,2)) + d(2);%0.148555884225591;
l3_0 = norm(S(:,3)-Q(:,3)) + d(3);%0.148837631359746;
l4_0 = norm(S(:,4)-Q(:,4)) + d(4);%0.16798554605892;
l5_0 = norm(S(:,5)-Q(:,5)) + d(5);%0.137341827458255;
l6_0 = norm(S(:,6)-Q(:,6)) + d(6);%0.137767712620365;

l0 = [l1_0 l2_0 l3_0 l4_0 l5_0 l6_0];

B = zeros(12,9);% because six eye thetas plus 3 head thetas
j31 = zeros(3,3);
j32 =zeros(3,3);
j33 =zeros(3,3);
j34 = zeros(3,3);
j41 =zeros(3,3);
j42 = zeros(3,3);
j43 = zeros(3,3);
j44 = zeros(3,3);

if strcmp(dimension,'prototype') == 1
    D_eye = 2*[0.02 0 0; 0 0.02 0; 0 0 0.02];
    D_head = 3*[0.55 0 0; 0 0.55 0; 0 0 0.55];
elseif strcmp(dimension,'real') == 1
    D_eye = 0.002*[0.02 0 0; 0 0.02 0; 0 0 0.02];
    D_head = 0.003*[0.55 0 0; 0 0.55 0; 0 0 0.55];
end
%local state
state = [eta_eye;eta_head;omega_eye;omega_head];

% Local state equationss
% eta_eye_dot = -skew(omega_eye)*eta_eye + delta_omega_eye;
% eta_head_dot = -skew(omega_head)*eta_head + delta_omega_head;
% omega_eye_dot = I_eye\(tau_eye - cross(omega_eye,I_eye*omega_eye));
% omega_head_dot = I_head\(tau_head - cross(omega_head,I_head*omega_head));

%2nd acceleration term diff w.r.t omega
diff_aux_eye = skew(omega_eye)*I_eye - skew(I_eye*omega_eye);
diff_aux_head = skew(omega_head)*I_head - skew(I_head*omega_head);


% Jacobian for A
j11 = -skew(omega_eye);
j12 = zeros(3,3);
j13 = eye(3);
j14 = zeros(3,3);
j21 = zeros(3,3);
j22 = -skew(omega_head);
j23 = zeros(3,3);
j24 = eye(3);
j33 = -I_eye\(D_eye + diff_aux_eye);
j34 =  I_eye\(D_eye*R_eye_world'*R_head_world);
%j43 = I_head\(D_eye*R_head_world'*R_eye_world);
%j44 = -I_head\(D_eye + D_head + diff_aux_head);

% Elements of the jacobian dependent on Q and S must be added 6 times to
% account for the 6 EOM
for i=1:6
    length = R_eye_world'*(R_head_world*S(:,i)-R_eye_world*Q(:,i));
    length_head = R_head_world'*(R_head_world*S(:,i) - R_eye_world*Q(:,i));
    
    tau_ela_wrt_eta_eye1 = (1/norm(length))*skew(R_eye_world'*R_head_world*S(:,i)) - (length*(norm(length))^-3)*S(:,i)'*(R_eye_world'*R_head_world)'*skew(Q(:,i));
    tau_ela_wrt_eta_eye2 = R_eye_world'*R_head_world*skew(S(:,i))*(1/norm(length)) + (length*(norm(length))^-3)*Q(:,i)'*R_eye_world'*R_head_world*skew(S(:,i));
    tau_ela_wrt_eta_head1 =(1/norm(length_head))*R_head_world'*R_eye_world*skew(Q(:,i)) - (length_head*(norm(length_head))^-3)*(S(:,i)'*R_head_world'*R_eye_world*skew(Q(:,i)));
    tau_ela_wrt_eta_head2 = skew(R_head_world'*R_eye_world*Q(:,i))*(1/norm(length_head)) + (length_head*(norm(length_head))^-3)*Q(:,i)'*(R_head_world'*R_eye_world)'*skew(S(:,i));
    
    j31 = j31 + I_eye\(skew(Q(:,i))*(skew(R_eye_world'*R_head_world*S(:,i))*(k/l0(i)) + (k/l0(i))*(r*theta(i) + d(i) - l0(i) )*tau_ela_wrt_eta_eye1));
    j32 = j32 - I_eye\(skew(Q(:,i))*((k/l0(i))*R_eye_world'*R_head_world*skew(S(:,i)) + (k/l0(i))*(r*theta(i) + d(i) - l0(i) )*tau_ela_wrt_eta_eye2));
    % j41 = j41 - I_head\(skew(S(:,i))*(((k/l0(i))*R_head_world'*R_eye_world*skew(Q(:,i))) + (k/l0(i))*(r*theta(i) + d(i) - l0(i) )*tau_ela_wrt_eta_head1));
    %j42 = j42 + I_head\(skew(S(:,i))*(((k/l0(i))*skew(R_head_world'*R_eye_world*Q(:,i))) + (k/l0(i))*(r*theta(i) + d(i) - l0(i) )*tau_ela_wrt_eta_head2));
    
    B(:,i) = [zeros(3,1); zeros(3,1); I_eye\skew(Q(:,i))*(k/l0(i))*r*(length/norm(length)); zeros(3,1) ];
    
end
%B(10:12,7:9) = I_head\eye(3);
%B(7:9,1:6)= I_eye\[1 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1];
j31 = j31 + I_eye\(D_eye*skew(R_eye_world'*R_head_world*omega_head));
j32 = j32 - I_eye\(D_eye*R_eye_world'*R_head_world*skew(omega_head));
%j41 = j41 - I_head\(D_eye*R_head_world'*R_eye_world*skew(omega_eye));
%j42 = j42 - I_head\(k_neck + D_eye*skew(R_head_world'*R_eye_world*omega_eye));
A = [j11 j12 j13 j14; j21 j22 j23 j24; j31 j32 j33 j34; j41 j42 j43 j44];

