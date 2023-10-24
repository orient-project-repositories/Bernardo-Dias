function [] = get_STD()

%% Plot combined Main Sequence plots

c = 0; %For continuous saccades
nonlin = 0; %For nonlinear model

if nonlin == 1 %Nonlinear
    zeroInitial= load('sacc(gridsacc)_Nacc_Eq_ww_24_70_1.2_0.07_29.4.mat');
    continuous = load('sacc(gridsacc)_Nacc_Eq_ww_Cont_24_70_1.2_0.07_29.4.mat');
else %linear
    zeroInitial= load('sacc_zeroinit_withtorsion_Bernard_test_set_complete2.mat');
%    zeroInitial= load('check_forces_PT=1.mat');
    continuous = load('continuous_saccade_Bernard_test_set_complete2.mat');
end

c_small = [1,6,7,15];
c_medium=[2,3,4,5,9,10,13,14,16,17,18,21,23,24];
c_large=[8,11,12,19,20,22];
z_small=[1,2,3,4,5,6,7,8];
z_medium=[9,11,13,15,17,19,21,23];
z_large=[10,12,14,16,18,20,22,24];

c_small_all = [];
c_medium_all = [];
c_large_all = [];
z_small_all = [];
z_medium_all = [];
z_large_all = [];
XYZ_zero = [];
XYZ_cont = [];

%Concatenate all trajectories
for i=1:length(zeroInitial.simresult)
    if nonlin == 1
        XYZ_zero = [XYZ_zero;zeroInitial.simresult(i).statevec(1:3,:)'];
        XYZ_cont = [XYZ_cont;continuous.simresult(i).statevec(1:3,:)'];
    else
        XYZ_zero = [XYZ_zero;zeroInitial.simresult(i).statevec(:,1:3)];
        XYZ_cont = [XYZ_cont;continuous.simresult(i).statevec(:,1:3)];
    end
end

%Get best fit plane
[n_z,~,p_z,~] = affine_fit(XYZ_zero);
[n_c,~,p_c,~] = affine_fit(XYZ_cont);

if c== 0
    %% Zero initial saccades
    for i = 1:length(z_small)
        if nonlin == 1
            zero_small(i).trajectory = zeroInitial.simresult(z_small(i)).statevec(1:3,:)';
            z_small_all = [z_small_all ;zero_small(i).trajectory(:,1:3)];
        else
            zero_small(i).trajectory = zeroInitial.simresult(z_small(i)).statevec(:,1:3);
            z_small_all = [z_small_all ;zero_small(i).trajectory(:,1:3)];
        end
        
    end
    
    for i = 1:length(z_medium)
        if nonlin == 1
            zero_medium(i).trajectory = zeroInitial.simresult(z_medium(i)).statevec(1:3,:)';
            z_medium_all = [z_medium_all ;zero_medium(i).trajectory(:,1:3)];
        else
            zero_medium(i).trajectory = zeroInitial.simresult(z_medium(i)).statevec(:,1:3);
            z_medium_all = [z_medium_all ;zero_medium(i).trajectory(:,1:3)];
        end
        
    end
    
    for i = 1:length(z_large)
        if nonlin == 1
            zero_large(i).trajectory = zeroInitial.simresult(z_large(i)).statevec(1:3,:)';
            z_large_all = [z_large_all ;zero_large(i).trajectory(:,1:3)];
        else
            zero_large(i).trajectory = zeroInitial.simresult(z_large(i)).statevec(:,1:3);
            z_large_all = [z_large_all ;zero_large(i).trajectory(:,1:3)];
        end
        
    end
    
    E_z_small=z_small_all-p_z;
    len_z_small=size(z_small_all,1);
    nn_z_s=ones(3,size(z_small_all,1)).*n_z;
    aux_zs = dot(E_z_small',nn_z_s);
    aux1_zs = mean(aux_zs);
    totalerror_z_s=sqrt(sum(abs(aux_zs-aux1_zs).^2)/len_z_small);
    
    E_z_medium=z_medium_all-p_z;
    len_z_medium=size(z_medium_all,1);
    nn_z_m=ones(3,size(z_medium_all,1)).*n_z;
    aux_zm = dot(E_z_medium',nn_z_m);
    aux1_zm = mean(aux_zm);
    totalerror_z_m=sqrt(sum(abs(aux_zm-aux1_zm).^2)/len_z_medium);
    
    E_z_large=z_large_all-p_z;
    len_z_large=size(z_large_all,1);
    nn_z_l=ones(3,size(z_large_all,1)).*n_z;
    aux_zl = dot(E_z_large',nn_z_l);
    aux1_zl = mean(aux_zl);
    totalerror_z_l=sqrt(sum(abs(aux_zl-aux1_zl).^2)/len_z_large);
    
    for i = 1:len_z_small
        aux = dot(E_z_small(i,:),n_z);      
        error_z_s(i) =abs(aux-aux1_zs);
    end
    for i = 1:len_z_medium
        aux = dot(E_z_medium(i,:),n_z);
        error_z_m(i) =abs(aux-aux1_zm);
    end
    for i = 1:len_z_large
        aux = dot(E_z_large(i,:),n_z);
        error_z_l(i) =abs(aux-aux1_zl);
    end
else
    %% Continuous saccades
    
    for i = 1:length(c_small)
        if nonlin ==1
            cont_small(i).trajectory = continuous.simresult(c_small(i)).statevec(1:3,:)';
            c_small_all = [c_small_all ;cont_small(i).trajectory(:,1:3)];
        else
            cont_small(i).trajectory = continuous.simresult(c_small(i)).statevec(:,1:3);
            c_small_all = [c_small_all ;cont_small(i).trajectory(:,1:3)];
        end
        
    end
    
    for i = 1:length(c_medium)
        if nonlin ==1
            cont_medium(i).trajectory = continuous.simresult(c_medium(i)).statevec(1:3,:)';
            c_medium_all = [c_medium_all ;cont_medium(i).trajectory(:,1:3)];
        else
            cont_medium(i).trajectory = continuous.simresult(c_medium(i)).statevec(:,1:3);
            c_medium_all = [c_medium_all ;cont_medium(i).trajectory(:,1:3)];
        end

    end
    
    for i = 1:length(c_large)
        if nonlin ==1
            cont_large(i).trajectory = continuous.simresult(c_large(i)).statevec(1:3,:)';
            c_large_all = [c_large_all ;cont_large(i).trajectory(:,1:3)];
        else
            cont_large(i).trajectory = continuous.simresult(c_large(i)).statevec(:,1:3);
            c_large_all = [c_large_all ;cont_large(i).trajectory(:,1:3)];
        end

    end
    
    
    
    E_c_small=c_small_all-p_c;
    len_c_small=size(c_small_all,1);
    nn_c_s=ones(3,size(c_small_all,1)).*n_c;
    aux_cs = dot(E_c_small',nn_c_s);
    aux1_cs = mean(aux_cs);
    totalerror_c_s=sqrt(sum(abs(aux_cs-aux1_cs).^2)/len_c_small);
    
    E_c_medium=c_medium_all-p_c;
    len_c_medium=size(c_medium_all,1);
    nn_c_m=ones(3,size(c_medium_all,1)).*n_c;
    aux_cm = dot(E_c_medium',nn_c_m);
    aux1_cm = mean(aux_cm);
    totalerror_c_m=sqrt(sum(abs(aux_cm-aux1_cm).^2)/len_c_medium);
    
    E_c_large=c_large_all-p_c;
    len_c_large=size(c_large_all,1);
    nn_c_l=ones(3,size(c_large_all,1)).*n_c;
    aux_cl = dot(E_c_large',nn_c_l);
    aux1_cl = mean(aux_cl);
    totalerror_c_l=sqrt(sum(abs(aux_cl-aux1_cl).^2)/len_c_large);
    
    %% Max deviation
    for i = 1:len_c_small
        aux = dot(E_c_small(i,:),n_c);
        error_c_s(i) =abs(aux-aux1_cs);
    end
    
    for i = 1:len_c_medium
        aux = dot(E_c_medium(i,:),n_c);
        error_c_m(i) =abs(aux-aux1_cm);
    end
    
    for i = 1:len_c_large
        aux = dot(E_c_large(i,:),n_c);
        error_c_l(i) =abs(aux-aux1_cl);
    end
    
end

%% Final results
if c==0
    total_std =atan([totalerror_z_s totalerror_z_m totalerror_z_l])*360/pi
    max_error = atan([max(error_z_s) max(error_z_m) max(error_z_l)])*360/pi
else
    total_std =atan([totalerror_c_s totalerror_c_m totalerror_c_l])*360/pi
    max_error = atan([max(error_c_s) max(error_c_m) max(error_c_l)])*360/pi
end



