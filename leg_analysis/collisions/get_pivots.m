function [pivots, theta] = get_pivots(k)
%GET_PIVOTS - calculate find theta for a given k and return corresponding
%             pivots
% Inputs:
%   k - scalar [0, 1]
%
% Outputs
%   pivots - pivot points 11x2 array

pivots_stroke = evalin('base', 'pivots_stroke');

% find min and max theta
thetas = pivots_stroke(:, 23);
theta_min = min(thetas);
theta_max = max(thetas);

theta_k = theta_max - k * (theta_max - theta_min);

% find nearest theta
[v, theta_idx] = min(abs(thetas - theta_k));
theta = thetas(theta_idx);

pivots = zeros(11, 2);

pivots(1, :) = [pivots_stroke(theta_idx, 1), pivots_stroke(theta_idx, 2)]; % A
pivots(2, :) = [pivots_stroke(theta_idx, 3), pivots_stroke(theta_idx, 4)]; % B
pivots(3, :) = [pivots_stroke(theta_idx, 5), pivots_stroke(theta_idx, 6)]; % C
pivots(4, :) = [pivots_stroke(theta_idx, 7), pivots_stroke(theta_idx, 8)]; % D
pivots(5, :) = [pivots_stroke(theta_idx, 9), pivots_stroke(theta_idx, 10)]; % F
pivots(6, :) = [pivots_stroke(theta_idx, 11), pivots_stroke(theta_idx, 12)]; % G
pivots(7, :) = [pivots_stroke(theta_idx, 13), pivots_stroke(theta_idx, 14)]; % H
pivots(8, :) = [pivots_stroke(theta_idx, 15), pivots_stroke(theta_idx, 16)]; % K
pivots(9, :) = [pivots_stroke(theta_idx, 17), pivots_stroke(theta_idx, 18)]; % L
pivots(10, :) = [pivots_stroke(theta_idx, 19), pivots_stroke(theta_idx, 20)]; % M
pivots(11, :) = [pivots_stroke(theta_idx, 21), pivots_stroke(theta_idx, 22)]; % P


end

