function [jacobian] = get_jacobian(theta, lc, lr)
%JACOBIAN - Calculated jacobian

% Inputs:
%    theta - Input angle        [rad]
%    lc - length of crank       [m]
%    lr - length of push rod    [m]
%
% Outputs:
%    jacobian - scalar value of jacobian 1x1
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

jacobian = - (lc^2 * sin(theta) * cos(theta)) / (lr^2 - lc^2 + (lc*sin(theta))^2)^0.5 - lc * cos(theta);

end
