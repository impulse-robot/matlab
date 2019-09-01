function [jacobian] = get_jacobian(theta, lc, lr, le)
%JACOBIAN - Calculated jacobian

% Inputs:
%    theta - Input angle        [rad]
%    lc - length of crank       [m]
%    lr - length of push rod    [m]
%    le - eccentric off set     [m]
%
% Outputs:
%    jacobian - scalar value of jacobian 1x1
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

jacobian = - (lc * sin(theta) * (lc * cos(theta) + le)) / (lr^2 - (lc*cos(theta) + le)^2)^0.5 - lc * cos(theta);

end
