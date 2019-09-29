 function [theta] = inverse_kinematics(z, lc, lr)
%inverse_kinematics - Calculate crank angle based on leg extension

% % Inputs:
%    z - leg extension in base frame    [m]
%    lc - length of crank               [m]
%    lr - length of push rod            [m]
%
% Outputs:
%    theta - crank angle                [rad]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

% theta = asin( (-lc^2 + lr^2 - z^2) / (2 * lc * z) );

theta = -asin((lc^2 - lr^2 + z^2)/(2*lc*z));

end