function [] = visualize_motion(t,Q)
%VISUALIZE_MOTION Visualize motion of robot
%   Visualization of the robot
%
%   Inputs:
%       t - k x 1 vector containing time
%
%       Q - K x 4 [phi, phi_dot, theta, theta_dot] vector containing all
%       state variables for different time steps
%

global L_F L_R L_C L_B R_F R_O M_F M_R M_C M_B I_F I_R I_C I_B

% joint position indices
FT_idx = 1;
RF_idx = 2;
CR_idx = 3;
MC_idx = 4;

time_increment = 0.01;

for i = 1 : size(t, 1)
    t_current = t(i);
    q_current = Q(i, :);
    
    joint_positions = get_joint_positions(q_current);
    FT = joint_positions(FT_idx, :);
    RF = joint_positions(RF_idx, :);
    CR = joint_positions(CR_idx, :);
    MC = joint_positions(MC_idx, :);
    L_B = 0.07;
    
    foot = [[FT(1); RF(1)], [FT(2); RF(2)]];
    pushrod = [[RF(1); CR(1)], [RF(2); CR(2)]];
    crank = [[CR(1); MC(1)], [CR(2); MC(2)]];
    body = [[MC(1) - L_B/2; MC(1) + L_B/2; MC(1) + L_B/2; MC(1) - L_B/2], [MC(2) + L_B/2; MC(2) + L_B/2; MC(2) - L_B/2; MC(2) - L_B/2]];
    
    clf
    
    % plot links
    plot(foot(:, 1), foot(:, 2), 'b', 'LineWidth', 3);
    hold on
    plot(pushrod(:, 1), pushrod(:, 2), 'b', 'LineWidth', 3);
    hold on
    plot(crank(:, 1), crank(:, 2), 'b', 'LineWidth', 3);
    hold on
    
    % plot body
    body_area =  fill(body(:, 1), body(:, 2), 'b', 'LineWidth', 3);
    set(body_area, 'facealpha', .3);
    
    % plot pivots
    scatter([FT(1), RF(1), CR(1), MC(1)], [FT(2), RF(2), CR(2), MC(2)], 75,...
        'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineWidth', 4);
    
    txt = {'Time:', ['t = ', num2str(t_current), 's'] };
    text(-0.08, 0.35, txt)
    
    axis equal
    grid on
    
    %     xlim([-0.20, 0.15]);
    %     ylim([-0.05, 0.35]);
    
    xlim([-0.45, 0.45]);
    ylim([-0.45, 0.45]);
    
    if (i > 1)
        time_increment = t(i) - t(i-1);
    else
        input('Enter to Start visualization \n');
    end
    
    pause(time_increment);
    
    
end

end

