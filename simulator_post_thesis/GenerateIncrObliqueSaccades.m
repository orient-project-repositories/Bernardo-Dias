function [test_goals] = GenerateIncrObliqueSaccades()

y1 = linspace(0.1,0.4,4);
z1 = linspace(0.1,0.4,4);

x = zeros(1,32);
y = [zeros(1,8) y1 -y1 y1 y1 -y1 -y1];
z = [z1 -z1 zeros(1,8) z1 -z1 z1 -z1];

test_goals = [x;y;z];
save('TestSetComplete.mat','test_goals');
figure()
hold on
xline(0);
yline(0);
for i = 1:length(test_goals)
    scatter(test_goals(2,i),test_goals(3,i));
    
    grid on
end