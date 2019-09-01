function pivots = calculate_pivots(joint_angles, link_lengths, link_angles)
%CALCULATE_PIVOTS - calculate the positions of the pivots
% Syntax:  pivots = visualize_lega(angles)
%
% Inputs:
%    joint_angles - joint angles stored in a nx7 array
%    link_lengths - link lengths stored in a 1x13 array
%    link_angles - link angles stored in a 1x6 array
%
% Outputs:
%    pivots - position of joints stored in an nx11x2
%
% Example:
%    calculate_pivots(angles)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

% store geometry
a_1 = link_lengths(1);
a_2 = link_lengths(2);
b_1 = link_lengths(3);
b_2 = link_lengths(4);
d_1 = link_lengths(8);
d_2 = link_lengths(9);
l_0 = link_lengths(10);
l_1 = link_lengths(11);
l_2 = link_lengths(12);
l_3 = link_lengths(13);

alpha_1 = link_angles(1);
beta_2 = link_angles(3);
delta_1 = link_angles(5);
epsilon_0 = link_angles(6);

pivots = zeros(size(joint_angles, 1), 11, 2);

% iterate over solved joint configurations
for row=1:size(joint_angles, 1)
    joint_angles_curr = joint_angles(row, :);
    theta_1 = joint_angles_curr(1);
    theta_2 = joint_angles_curr(2);
    theta_3 = joint_angles_curr(3);
    theta_4 = joint_angles_curr(4);
    theta_5 = joint_angles_curr(5);
    theta_7 = joint_angles_curr(7);
    
    
    pivots(row, 1, 1) = -l_0 * cos(epsilon_0); % A
    pivots(row, 1, 2) = -l_0 * sin(epsilon_0); % A
    pivots(row, 2, 1) = 0; % B
    pivots(row, 2, 2) = 0; % B
    pivots(row, 5, 1) = a_1 * cos(theta_1); % F
    pivots(row, 5, 2) = a_1 * sin(theta_1); % F
    pivots(row, 7, 1) = pivots(row, 5, 1) + l_1 * cos(theta_2); % H
    pivots(row, 7, 2) = pivots(row, 5, 2) + l_1 * sin(theta_2); % H
    pivots(row, 3, 1) = pivots(row, 7, 1) + b_2 * cos(theta_3); % C
    pivots(row, 3, 2) = pivots(row, 7, 2) + b_2 * sin(theta_3); % C
    pivots(row, 4, 1) = a_2 * cos(theta_1 + alpha_1); % D
    pivots(row, 4, 2) = a_2 * sin(theta_1 + alpha_1); % D
    pivots(row, 6, 1) = pivots(row, 4, 1) + l_3 * cos(theta_7); % G
    pivots(row, 6, 2) = pivots(row, 4, 2) + l_3 * sin(theta_7); % G
    pivots(row, 9, 1) = pivots(row, 6, 1) + b_1 * cos(theta_3 - beta_2); % L
    pivots(row, 9, 2) = pivots(row, 6, 2) + b_1 * sin(theta_3 - beta_2); % L
    pivots(row, 10, 1) = pivots(row, 9, 1) + d_1 * cos(theta_4); % M
    pivots(row, 10, 2) = pivots(row, 9, 2) + d_1 * sin(theta_4); % M
    pivots(row, 8, 1) = pivots(row, 10, 1) + l_2 * cos(theta_5); % K
    pivots(row, 8, 2) = pivots(row, 10, 2) + l_2 * sin(theta_5); % K
    pivots(row, 11, 1) = pivots(row, 9, 1) - d_2 * cos(theta_4 - delta_1); % P_0
    pivots(row, 11, 2) = pivots(row, 9, 2) - d_2 * sin(theta_4 - delta_1); % P_0
    
end

end
