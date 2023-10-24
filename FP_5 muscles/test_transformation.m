function [] = test_transformation()

theta = 2*pi*rand;
phi =  -pi/2 + pi*rand;
rs = 0.187689/2; %radius of the sphere

% [x3,y3,z3] = spherical_2_cartesian(theta,phi,rs);
% test_point = [x3 y3+0.0593695 z3];

[x3,y3,z3] = spherical_2_cartesian(theta,phi,rs);
test_point2 = [x3 y3 z3];
%test_point2 = world_to_head(test_point,[0 0 0 0]);
test_point = head_to_world(test_point2,[0 0 0 0]);
[x,y,z] = sphere(50);
rc = 0.055/2;
[xc,yc,zc] = cylinder(rc);
x2 = x*rs;
y2 = y*rs;
z2 = z*rs;
%Plot 2D and 3D points on circle/spherical surface
center_s = 0.201422 + 0.0593695;
height_c = 0.201422; 
mycolors = [0.9 0.9 0.9; 0.9 0.9 0.9; 0.9 0.9 0.9];
lightGrey = 0.8*[1 1 1];
figure()
surf(x2,y2,z2+center_s,'FaceColor', 'none','EdgeColor',lightGrey);
hold on
surf(xc,yc,zc*height_c);
plotcube([0.09,0.1,0.01],[-0.05 -0.05 0],1,[0 0 0]);
plot3(test_point(1), test_point(2) ,test_point(3),'x');
colormap(mycolors)
xlim([-0.4 0.4]);
ylim([-0.4 0.4]);
zlim([0 0.5]);
figure()
viscircles([0 0],rs);
hold on 
plot(test_point(1),test_point(2),'x');

