function [direction] = get_direction(Q,I0)
len_3d = zeros(length(I0),size(I0,2));
norm_len = zeros(length(I0),1);
direction = zeros(length(I0),size(I0,2));
for i = 1:length(I0)
    len_3d(i,:) = Q(i,:)-I0(i,:);
    norm_len(i) = norm(len_3d(i,:));
    direction(i,:) = len_3d(i,:)/norm_len(i);
end