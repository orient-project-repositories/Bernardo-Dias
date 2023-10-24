function [] = run_several_optimization()
clear all
close all
for i=1:2
     [solution(:,i),max_force(i)] = get_insertion_points();
end

assignin('base','sev_solutions',solution);
assignin('base','list_max_forces',max_force);