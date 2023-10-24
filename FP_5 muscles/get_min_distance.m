function [real_dist] = get_min_distance(Q,I) %Q and I are the insertion points dim = 10x3

min_dist = zeros(length(I),1);
real_dist = [];
% Got to avoid comparing a vector with itself and no repeating comparisons
for j=1:length(Q)  
    for k=j+1:length(I)
        min_dist(k-1) = distLinSeg(Q(j,:),I(j,:),Q(k,:),I(k,:));
    end
    
    real_dist =[real_dist;min_dist];
    
end
%real_dist(real_dist==0)=[];
% % x = [1, 2, 3, 4];
% % y = [1, 3, 4, 3]
% % for xIndex = 1 : length(x)
% %   for yIndex = xIndex : length(y)
% %     distances(xIndex, yIndex) = sqrt((x(xIndex)-x(yIndex))^2 + (y(xIndex)-y(yIndex))^2)
% %   end
% % end
% % distances  % Report to the command window.