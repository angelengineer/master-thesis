function T = dh_trans(a, alpha, d, theta)
% DH_TRANSFORM_MODIFIED Calculate transformation matrix using modified DH parameters
%
% Inputs:
%   a     - Link length (common normal distance)
%   alpha - Link twist (rotation around common normal)
%   d     - Link offset (distance along previous z)
%   theta - Joint angle (rotation around previous z)
%
% Output:
%   T     - 4x4 homogeneous transformation matrix

% Validate input dimensions
if ~isscalar(a) || ~isscalar(alpha) || ~isscalar(d) || ~isscalar(theta)
    error('All input parameters must be scalars');
end

% Calculate trigonometric functions
ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

% Construct the transformation matrix using modified DH convention
T = [ ct    -st    0     a;
      st*ca  ct*ca -sa  -sa*d;
      st*sa  ct*sa  ca   ca*d;
       0      0     0     1   ];
end