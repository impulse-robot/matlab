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
        i = evalin('base', 'i');
        f = (i * torque_curve(get_theta_dot(y, y_dot) * i)) / get_jacobian(y);
    else
        f = 0;
    end
end
