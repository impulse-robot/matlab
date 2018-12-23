function [tau] = torque_curve(omega_dot)
%TORQUE_CURVE - Models the torque rpm curve
% Syntax:  tau = torque_curve(theta_dot)
%
% Inputs:
%    omega_dot - rotational velocity of motor shaft [rad/s]
%                = theta_dot / i
%
% Outputs:
%    tau - motor output torque
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none   

% TODO determine motor torque curve
max_torque = evalin('base', 'tau'); % [Nm]
max_power = 300; % [W]
if ((omega_dot * max_torque) < max_power)
    tau = max_torque;
else
    tau = max_power / omega_dot;
end

if (60 * omega_dot / (2*pi) > 30000)
        tau = 0;
end
    
end

