function [th,r,z1] = cartesian_2_cylindrical(vec)
%CART2POL Transform Cartesian to polar coordinates.
%   [TH,R] = CART2POL(X,Y) transforms corresponding elements of data
%   stored in Cartesian coordinates X,Y to polar coordinates (angle TH
%   and radius R).  The arrays X and Y must be the same size (or
%   either can be scalar). TH is returned in radians. 
%
%   [TH,R,Z] = CART2POL(X,Y,Z) transforms corresponding elements of
%   data stored in Cartesian coordinates X,Y,Z to cylindrical
%   coordinates (angle TH, radius R, and height Z).  The arrays X,Y,
%   and Z must be the same size (or any of them can be scalar).  TH is
%   returned in radians.
%
%   Class support for inputs X,Y,Z:
%      float: double, single
%
%   See also CART2SPH, SPH2CART, POL2CART.

%   Copyright 1984-2005 The MathWorks, Inc. 
x = vec(1);
y = vec(2);
z = vec(3);
th = atan2(-x,z);
r = 0.055/2;%radius of the cylinder
z1 = -y;