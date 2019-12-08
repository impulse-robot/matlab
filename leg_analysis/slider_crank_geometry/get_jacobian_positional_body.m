function [jacobian] = get_jacobian_positional_body(phi, theta)
%GET_JACOBIAN_POSITIONAL_BODY calculate positional jacobian for body in
%inertial frame
%
%   Inputs:
%       phi - tilt angle of robot
%
%       theta  - output link angle of robot
%
%   Output:
%       jacobian - postional jacobian of body (2 x 2 matrix)
%

global L_F L_R L_C

jacobian = [-(L_F + L_R * sqrt(1 - (L_C / L_R * cos(theta))^2) + L_C * sin(theta)) * cos(phi), (-L_C^2 * sin(theta) * cos(theta) * sin(phi)) / (L_R * sqrt(1 - (L_C / L_R * cos(theta))^2)) - L_C * cos(theta) * sin(phi);
    -(L_F + L_R * sqrt(1 - (L_C / L_R * cos(theta))^2) + L_C * sin(theta)) * sin(phi), (L_C^2 * sin(theta) * cos(theta) * cos(phi)) / (L_R * sqrt(1 - (L_C / L_R * cos(theta))^2)) + L_C * cos(theta) * cos(phi)];

end

