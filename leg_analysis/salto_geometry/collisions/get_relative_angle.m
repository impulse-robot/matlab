function [rel_angle] = get_relative_angle(theta)
%GET_RELATIVE_ANGLE - calculate relative angle between shin and rod 
%                     for collision analysis
%             pivots
% Inputs:
%   theta - scalar
%
% Outputs
%   rel_angle - relative angle in rad

pivots_stroke = evalin('base', 'pivots_stroke');
pivots_cad = evalin('base', 'pivots_cad');

thetas = pivots_stroke(:, 23);
[v, theta_idx] = min(abs(thetas - theta));

H_0 = pivots_cad(7, :);
G_0 = pivots_cad(6, :);
HG_0 = G_0 - H_0;
shin_angle_0 = atan2(HG_0(2), HG_0(1));

K_0 = pivots_cad(8, :);
A_0 = pivots_cad(1, :);
AK_0 = K_0 - A_0;
rod_angle_0 = atan2(AK_0(2), AK_0(1));

rel_angle_0 = rod_angle_0 - shin_angle_0;

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

H_theta = pivots(7, :);
G_theta = pivots(6, :);
HG_theta = G_theta - H_theta;
shin_angle_theta = atan2(HG_theta(2), HG_theta(1));

K_theta = pivots(8, :);
A_theta = pivots(1, :);
AK_theta = K_theta - A_theta;
rod_angle_theta = atan2(AK_theta(2), AK_theta(1));

rel_angle_theta = rod_angle_theta - shin_angle_theta;

rel_angle = rel_angle_theta;

end

