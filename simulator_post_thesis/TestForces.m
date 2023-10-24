%% Test static forces
close all
WA = 0; %test done on wrap around results
LT = 0;%test done on lookup table results
grid = 0; % get only singular force values for force
tau = 0; % to check elastic torque

if WA == 0
    PT =  [ 2.0189  1.9630 1.9811 2.0370 1.9715  2.0285];
    %PT = [ 1.0191 1.0000 1.0000 1.0403 1.0000 1.0289];%% Initial thetas
else
%     PT = [1.0016 1.1072 0.9984 0.8928 0.9857 1.0143];
    PT = [2.0032  2.2143  1.9968  1.7857 1.9714 2.0286];
    % [ 1.2019 1.3286 1.1981 1.0714 1.1828 1.2172] % initial thetas for WA
    % plot elastic torque instead of force
end
nr_iterations = 9; % for increasingly higher horizontal orientations
if LT == 0
    nr_iterations = 1;
end
if grid == 1
    r = load('grid.mat');
    for i =1:length(r.orientations)
        goal = r.orientations(i,:);
        [eqRot,final_forces] = CheckForces(goal,PT,WA,LT,grid,tau);
        result(i).orientation = eqRot;
        result(i).final_forces = final_forces;
    end
else
    for i = 1:nr_iterations
        h = -0.7+ i*0.15;
        goal = [0 0 h];
        [eqRot,final_forces] = CheckForces(goal,PT,WA,LT,grid,tau);
        result(i).orientation = eqRot;
        result(i).final_forces = final_forces;
        
    end
end