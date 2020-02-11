function [] = visualize_motion(t,thetas, mp)
%VISUALIZE_MOTION Visualize motion of robot
%   Visualization of the robot
%
%   Inputs:
%       t - k x 1 vector containing time
%
%       theta - k x 1 vector containg theta angle
%
%       mp - struct containing model parameters


time_increment = 0.01;
time_scaling = 5;

desired_size = 50;
% downsample vector
if (size(t, 2) > desired_size)
    downsample_factor = floor(size(t, 2) / desired_size);
    t = downsample(t, downsample_factor);
    thetas = downsample(thetas, downsample_factor);
end

figure('name', 'Jump Animation');

for i = 1 : size(t, 2)
    t_current = t(i);
    theta = thetas(i);
    
    FT = [0; 0];
    RF = [0; mp.length_foot];
    CR = [-mp.length_crank * cos(theta); mp.length_foot + mp.length_pushrod * ...
        sqrt(1 - (mp.length_crank/mp.length_pushrod * cos(theta))^2)];
    MC = [0; mp.length_foot + mp.length_pushrod * ...
        sqrt(1 - (mp.length_crank/mp.length_pushrod * cos(theta))^2) + mp.length_crank * sin(theta)];
    
    foot = [[FT(1); RF(1)], [FT(2); RF(2)]];
    pushrod = [[RF(1); CR(1)], [RF(2); CR(2)]];
    crank = [[CR(1); MC(1)], [CR(2); MC(2)]];
    body = [[MC(1) - mp.length_body/2; MC(1) + mp.length_body/2; MC(1) + ...
        mp.length_body/2; MC(1) - mp.length_body/2], [MC(2) + ...
        mp.length_body/2; MC(2) + mp.length_body/2; MC(2) - ...
        mp.length_body/2; MC(2) - mp.length_body/2]];
    
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
    
    pause(time_increment * time_scaling);
    
    
end

end

