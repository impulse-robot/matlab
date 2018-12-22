function [] = visualize_thrusters(pivots, l_pitch, l_roll_yaw, rod_angle)
%VISUALIZE_THRUSTERS Visualization of the leg and the thrusters
%   
% Inputs:
%    pivots - pivots points stored in a 11x2 array

    A = [pivots(1), pivots(2)];
    B = [pivots(3), pivots(4)];
    C = [pivots(5), pivots(6)];
    D = [pivots(7), pivots(8)];
    F = [pivots(9), pivots(10)];
    G = [pivots(11), pivots(12)];
    H = [pivots(13), pivots(14)];
    K = [pivots(15), pivots(16)];
    L = [pivots(17), pivots(18)];
    M = [pivots(19), pivots(20)];
    P_0 = [pivots(21), pivots(22)];
    
    cog = B;
    body_height = 0.1;
    body_length = 0.12;
    
    crank = [[B(1); D(1); F(1)], [B(2); D(2); F(2)]];
    shin = [[H(1); G(1); L(1); C(1)], [H(2); G(2); L(2); C(2)]];
    foot = [[M(1); L(1); P_0(1)], [M(2); L(2); P_0(2)]];
    rod = [[A(1); K(1); C(1)], [A(2); K(2); C(2)]];
    crank_link_upper = [[F(1); H(1)], [F(2); H(2)]];
    crank_link_lower = [[D(1); G(1)], [D(2); G(2)]];
    achilles_link = [[K(1); M(1)], [K(2); M(2)]];
    
    % plot body
    pos_body = [cog(1) - body_length / 2, cog(2) - body_height / 2, body_length, body_height];
    rectangle('Position', pos_body, 'Curvature', 0.3, 'FaceColor', [0.6, 0.6, 0.6], 'LineWidth', 3);
    hold on
    
    % plot terniary links
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
    scatter(pivots(1:2:end), pivots(2:2:end), 75, 'k', 'LineWidth', 4);
    
    % plot thrusters
    pitch_thruster = cog + [0, l_pitch];
    roll_thruster_1 = cog + [l_roll_yaw * sin(rod_angle), l_roll_yaw * cos(rod_angle)];
    roll_thruster_2 = cog + [l_roll_yaw * sin(-rod_angle), l_roll_yaw * cos(rod_angle)];
    
    plot([cog(1), pitch_thruster(1)], [cog(2), pitch_thruster(2)], 'k', 'LineWidth', 4);
    hold on
    
    plot([cog(1), roll_thruster_1(1)], [cog(2), roll_thruster_1(2)], 'k', 'LineWidth', 4);
    hold on
    
    plot([cog(1), roll_thruster_2(1)], [cog(2), roll_thruster_2(2)], 'k', 'LineWidth', 4);
    hold on
   
    pitch_motor_height = 16.3 / 1000;
    pitch_motor_diameter = 14.4 / 1000;
    roll_motor_height = 8.2 / 1000;
    roll_motor_diameter = 9 / 1000;
    
    pos_pitch = [pitch_thruster(1) - pitch_motor_height / 2, pitch_thruster(2) - pitch_motor_diameter / 2, pitch_motor_height, pitch_motor_diameter];
    pos_roll_1 = [roll_thruster_1(1) - roll_motor_height / 2, roll_thruster_1(2) - roll_motor_diameter / 2, roll_motor_height, roll_motor_diameter];
    pos_roll_2 = [roll_thruster_2(1) - roll_motor_height / 2, roll_thruster_2(2) - roll_motor_diameter / 2, roll_motor_height, roll_motor_diameter];
    
    rectangle('Position', pos_pitch, 'Curvature', 0.3, 'FaceColor', 'r', 'LineWidth', 2);
    rectangle('Position', pos_roll_1, 'Curvature', 0.3, 'FaceColor', [1, 0.6, 0.2], 'LineWidth', 2);
    rectangle('Position', pos_roll_2, 'Curvature', 0.3, 'FaceColor', [1, 0.6, 0.2], 'LineWidth', 2);
    
    % plot propellers
    
    propeller_diameter = 50.8 / 1000;
    propeller_thickness = 5 / 1000;
    pos_propeller_pitch = [pitch_thruster(1) + pitch_motor_height / 2, pitch_thruster(2) - propeller_diameter / 2, propeller_thickness, propeller_diameter];
    rectangle('Position', pos_propeller_pitch, 'Curvature', 0.1, 'FaceColor', 'r', 'LineWidth', 2);
    title('Thruster Visualization')
    xlabel('x[m]')
    ylabel('y[m]')
    axis equal

    grid on



end

