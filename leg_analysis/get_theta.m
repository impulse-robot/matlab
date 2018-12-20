function [theta] = get_theta(y)
%GET_THETA - Calculates the output shaft angle theta based on y
% Syntax:  theta = get_theta(y, y_dot)
%
% Inputs:
%    y - vertical position of robot [m]
%
% Outputs:
%    theta - output shaft angle [rad]
%
% Other m-files required: jump_height.m
% Subfunctions: none
% MAT-files required: none
    
    y_max = evalin('base', 'y_max');
    forward_kinematics = evalin('base', 'forward_kinematics');
    
    if (y < y_max)
        theta = interp1(forward_kinematics(:, 2), ...
            forward_kinematics(:, 1), y);
    else
        theta = interp1(forward_kinematics(:, 2), ...
            forward_kinematics(:, 1), y_max); 
    end
   
end