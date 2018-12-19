function [f] = ground_force(y, y_dot)
%GROUND_FORCE - Calculates the reaction foot force
% Syntax:  f = ground_force(y, y_dot)
%
% Inputs:
%    y - vertical position of robot [m]
%    y_dot - vertical velocity of robot [m/s]
%
% Outputs:
%    f - reaction force in vertical direction [N]
%
% Other m-files required: jump_height.m
% Subfunctions: none
% MAT-files required: none   

    if (y < evalin('base', 'y_max'))
        forward_kinematics = evalin('base', 'forward_kinematics');
        differential_kinematics = evalin('base', 'differential_kinematics');
        jacobian = interp1(differential_kinematics(:,1), ...
        differential_kinematics(:,2), interp1(forward_kinematics(:, 2), ...
        forward_kinematics(:, 1), y));
        f = (evalin('base', 'i') * torque_curve(y_dot / jacobian)) / jacobian;
    else
        f = 0;
    end
end
