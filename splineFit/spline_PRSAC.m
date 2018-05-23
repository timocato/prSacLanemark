% spline fitting 
% This is a third-party 2d spline fitting packet, thanks to reference [1]
% - [1] ??
% - modified by Yingmao Li
% input : 
%   x, y : measurement vector of size {1 x n}
%   breaks       : number of break points of the spline
%   constrains   : constrains, can be number of spline order
%   robust_factor: robust factor, [0, 1] 
% output:
%   params       : parameters of spline
function params = spline_PRSAC(x, y, breaks, constraints, robust_factor)
    params = splinefit(x, y, breaks,constraints, robust_factor);
end