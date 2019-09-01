function [theta_dot] = get_theta_dot(y, y_dot)
%GET_THETA_DOT - Calculates the output shaft angular velocity 
% Syntax:  theta_dot = get_theta_dot(y, y_dot)
%
% Inputs:
%    y - vertical position of robot [m]
%    y_dot - vertical velocity of robot [m/s]
%
% Outputs:
%    theta_dot - output shaft angular velocity [rad/s]
%
% Other m-files required: jump_height.m
% Subfunctions: none
% MAT-files required: none
    
    if (y < evalin('base', 'y_max'))
        theta_dot = y_dot / get_jacobian(y);
    else
        theta_dot = 0;
    end
   
end