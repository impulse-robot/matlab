function [] = visualize_leg(pivots)
%VISUALIZE_LEG - visualize the leg
% Syntax: visualize_leg(pivots)
%
% Inputs:
%    pivot points - pivots points stored in a nx11x2 array
%
% Outputs:
%    None
%
% Example: 
%    visualize_leg(pivots)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none

end_effector = [];
scaling = evalin('base', 'scaling');

figure('Name', 'Leg Visualization')

for row = 1:size(pivots, 1)
    A = pivots(row, 1, :);
    B = pivots(row, 2, :);
    C = pivots(row, 3, :);
    D = pivots(row, 4, :);
    F = pivots(row, 5, :);
    G = pivots(row, 6, :);
    H = pivots(row, 7, :);
    K = pivots(row, 8, :);
    L = pivots(row, 9, :);
    M = pivots(row, 10, :);
    P_0 = pivots(row, 11, :);
    
    input_angle = atan2(F(2), F(1));
    end_effector = [end_effector; P_0];
    
    crank = [[B(1); D(1); F(1)], [B(2); D(2); F(2)]];
    shin = [[H(1); G(1); L(1); C(1)], [H(2); G(2); L(2); C(2)]];
    foot = [[M(1); L(1); P_0(1)], [M(2); L(2); P_0(2)]];
    rod = [[A(1); K(1); C(1)], [A(2); K(2); C(2)]];
    crank_link_upper = [[F(1); H(1)], [F(2); H(2)]];
    crank_link_lower = [[D(1); G(1)], [D(2); G(2)]];
    achilles_link = [[K(1); M(1)], [K(2); M(2)]];
    
    % plot terniary links
    clf;
    crank_area = fill(crank(:, 1), crank(:, 2), 'b', 'LineWidth', 3);
    set(crank_area,'facealpha',.3)
    hold on
    shin_area = fill(shin(:, 1), shin(:, 2), 'b', 'LineWidth', 3);
    set(shin_area,'facealpha',.3)
    hold on
    foot_area = fill(foot(:, 1), foot(:, 2), 'b', 'LineWidth', 3);
    set(foot_area,'facealpha',.3)
    hold on
    rod_area = fill(rod(:, 1), rod(:, 2), 'b', 'LineWidth', 3);
    set(rod_area,'facealpha',.3)
    hold on

    % plot links
    plot(crank_link_upper(:, 1), crank_link_upper(:, 2), 'b', 'LineWidth', 4);
    hold on
    plot(crank_link_lower(:, 1), crank_link_lower(:, 2), 'b', 'LineWidth', 4);
    hold on
    plot(achilles_link(:, 1), achilles_link(:, 2), 'b', 'LineWidth', 4);
    hold on

    % plot pivots
    scatter(pivots(row, :, 1), pivots(row, :, 2), 75, 'k', 'LineWidth', 4);
    hold on


    scatter(end_effector(:, 1), end_effector(:, 2), 30, 'r', 'LineWidth', 2); 
    hold on
    stride_length = abs(P_0(2) - end_effector(1, 2)) * 100;
    txt = {'Stride Length:', ['\Deltal = ', num2str(stride_length), 'cm'], '', ...
        'Input angle:', ['\theta = ', num2str(input_angle), 'rad']};
    text(0.08, -0.2, txt)
    
    title('Leg Kinematics')
    xlabel('x[m]')
    ylabel('y[m]')
    axis equal
    xlim([-0.15 * scaling, 0.15 * scaling])
    ylim([-0.25 * scaling, 0.05 * scaling])
    grid on
    pause(0.001)

    if (row == 1)
        disp('-- Press any button to start leg visualization')
        waitforbuttonpress
        disp('-- Visualizing leg...')
    end
    
end

pivots = flipud(pivots);

for row = 1:size(pivots, 1)
    A = pivots(row, 1, :);
    B = pivots(row, 2, :);
    C = pivots(row, 3, :);
    D = pivots(row, 4, :);
    F = pivots(row, 5, :);
    G = pivots(row, 6, :);
    H = pivots(row, 7, :);
    K = pivots(row, 8, :);
    L = pivots(row, 9, :);
    M = pivots(row, 10, :);
    P_0 = pivots(row, 11, :);
    
    input_angle = atan2(F(2), F(1));
    end_effector = [end_effector; P_0];
    
    crank = [[B(1); D(1); F(1)], [B(2); D(2); F(2)]];
    shin = [[H(1); G(1); L(1); C(1)], [H(2); G(2); L(2); C(2)]];
    foot = [[M(1); L(1); P_0(1)], [M(2); L(2); P_0(2)]];
    rod = [[A(1); K(1); C(1)], [A(2); K(2); C(2)]];
    crank_link_upper = [[F(1); H(1)], [F(2); H(2)]];
    crank_link_lower = [[D(1); G(1)], [D(2); G(2)]];
    achilles_link = [[K(1); M(1)], [K(2); M(2)]];
    
    % plot terniary links
    clf;
    crank_area = fill(crank(:, 1), crank(:, 2), 'b', 'LineWidth', 3);
    set(crank_area,'facealpha',.3)
    hold on
    shin_area = fill(shin(:, 1), shin(:, 2), 'b', 'LineWidth', 3);
    set(shin_area,'facealpha',.3)
    hold on
    foot_area = fill(foot(:, 1), foot(:, 2), 'b', 'LineWidth', 3);
    set(foot_area,'facealpha',.3)
    hold on
    rod_area = fill(rod(:, 1), rod(:, 2), 'b', 'LineWidth', 3);
    set(rod_area,'facealpha',.3)
    hold on

    % plot links
    plot(crank_link_upper(:, 1), crank_link_upper(:, 2), 'b', 'LineWidth', 4);
    hold on
    plot(crank_link_lower(:, 1), crank_link_lower(:, 2), 'b', 'LineWidth', 4);
    hold on
    plot(achilles_link(:, 1), achilles_link(:, 2), 'b', 'LineWidth', 4);
    hold on

    % plot pivots
    scatter(pivots(row, :, 1), pivots(row, :, 2), 75, 'k', 'LineWidth', 4);
    hold on


    scatter(end_effector(:, 1), end_effector(:, 2), 30, 'r', 'LineWidth', 2); 
    hold on
    stride_length = abs(P_0(2) - end_effector(1, 2)) * 100;
    txt = {'Stride Length:', ['\Deltal = ', num2str(stride_length), 'cm'], '', ...
        'Input angle:', ['\theta = ', num2str(input_angle), 'rad']};
    text(0.08, -0.2, txt)
    
    title('Leg Kinematics')
    xlabel('x[m]')
    ylabel('y[m]')
    axis equal
    xlim([-0.15 * scaling, 0.15 * scaling])
    ylim([-0.25 * scaling, 0.05 * scaling])
    grid on
    pause(0.001)
    
end

disp('-- Finished visualization')

end

