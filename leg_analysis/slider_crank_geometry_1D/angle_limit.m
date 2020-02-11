function [value,isterminal, direction] = angle_limit(t,y)
% angle limit - used to terminate ODE solver, once limit is reached
%               intergration stops
%
%   Input:
%       t - current time (1x1)
%       y - current state (2x1)
%
%   Output:
%       value - expression describing the event, event occurs when value is
%               equal to zero
%       isterminal - 1 if the integration is to terminate, else 0
%       direction - 0 if all zeros are to be located, +1 located only zeros
%                   where event function increase and -1 only when event
%                   function decreases

angle_limit = pi / 2;
value = y(1) - angle_limit;
isterminal = 1;
direction = 1;


end

