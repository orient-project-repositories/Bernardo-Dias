function hh = xline(varargin)

args = varargin;
h = matlab.graphics.internal.xyzline('x',args);

if nargout > 0
    hh = h;
end
end