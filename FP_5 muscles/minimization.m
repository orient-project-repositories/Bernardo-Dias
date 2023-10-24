function [min_force] = minimization(force,points,numb,thetas) %make it global
len= get_len(points,thetas,numb);
min_force = norm(len)^2 + max(force)^2; %Force is a vector that comprises the average of the force produced by all the muscles
