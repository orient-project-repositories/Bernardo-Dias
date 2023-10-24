function [direction] = get_direction(Q,I0)

for i = 1:length(I)
    len_3d(i,:) = Q(i,:)-I0(i,:);
    norm_len(i) = norm(len_3d(i,:));
    direction(i,:) = len_3d(i,:)/norm_len(i);
end