function [az,elev,r] = cartesian_2_spherical(vec)
%CART2SPH Transform Cartesian to spherical coordinates.
%   [TH,PHI,R] = CART2SPH(X,Y,Z) transforms corresponding elements of
%   data stored in Cartesian coordinates X,Y,Z to spherical
%   coordinates (azimuth TH, elevation PHI, and radius R).  The arrays
%   X,Y, and Z must be the same size (or any of them can be scalar).
%   TH and PHI are returned in radians.
%
%   TH is the counterclockwise angle in the xy plane measured from the
%   positive x axis.  PHI is the elevation angle from the xy plane.
%
%   Class support for inputs X,Y,Z:
%      float: double, single
%
%   See also CART2POL, SPH2CART, POL2CART.

%   Copyright 1984-2005 The MathWorks, Inc. 
x = vec(1);
y = vec(2);
z = vec(3);
hypotxy = hypot(x,z);
r = 0.187689/2; %radius of the sphere
elev = atan2(-y,hypotxy);
az = atan2(z,x);
