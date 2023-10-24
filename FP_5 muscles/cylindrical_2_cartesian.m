function [x2,y2,z2] = cylindrical_2_cartesian(th,r,z)

x2 = -r*sin(th);
y2 = -z;
z2 = r*cos(th);
