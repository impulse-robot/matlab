function [jacobian] = get_jacobian(y)
%JACOBIAN - Calculated jacobian by interpolating forward and differential
%kinematics

% Inputs:
%    y - vertical position of robot [m]
%
% Outputs:
%    jacobian - scalar value of jacobian 1x1
%
% Other m-files required: jump_height.m
% Subfunctions: none
% MAT-files required: none   

    differential_kinematics = evalin('base', 'differential_kinematics');
    jacobian = interp1(differential_kinematics(:,1), ...
        differential_kinematics(:,2), get_theta(y));

end

