function [z] = forward_kinematics(theta, lc, lr)
%forward_kinematics - Calculate foot position based on crank angle

% Inputs:
%    theta - Input angle        [rad]
%    lc - length of crank       [m]
%    lr - length of push rod    [m]
%
% Outputs:
%    z - foot position in base frame
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

z = - lc*sin(theta) - (lr^2 - lc^2 + (lc * cos(theta))^2)^0.5;

end
