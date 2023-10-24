function [min_force] = force_min(force) %make it global

min_force = max(force)^2; %Force is a vector that comprises the average of the force produced by all the muscles

