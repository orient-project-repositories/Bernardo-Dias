%% Make a matrix to simulate Hepp85 results

horizontal = linspace(-0.1744,0.52,5)';
for i = 1:7
    vertical(i*5-4:i*5,:) = repelem(-0.6944+i*0.1744,5)';
end
% orientations = [zeros(35,1) vertical repmat(horizontal,7,1)]
orientations = [zeros(35,1) vertical repmat(horizontal,7,1)];
save('grid.mat','orientations');