function [y_dd] = y_dot_dot(y, y_dot)
%Y_DOT_DOT - second derivative used for differential equation of jumping
%height
% Syntax:  y_dd = y_dot_dot(y, y_dot)
%
% Inputs:
%    y - vertical position of robot [m]
%    y_dot - vertical velocity of robot [m/s]
%
% Outputs:
%    y_dd - vertical acceleration of robot [m/s^2]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

%     disp(ground_force(y, y_dot));
    y_dd = - evalin('base', 'g') / evalin('base', 'm') + ...
    ground_force(y, y_dot) / evalin('base', 'm');
end