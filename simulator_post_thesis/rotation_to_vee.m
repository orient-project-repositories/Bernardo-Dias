function vee_matrix = rotation_to_vee(rotation_matrix)

    a = logm(rotation_matrix);
    vee_matrix = [a(3,2); a(1,3); a(2,1)];
   