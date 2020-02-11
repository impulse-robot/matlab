function [] = visualize_jump_analysis(t, t_lin, theta, theta_lin, theta_dot, theta_dot_lin, ...
    body_velocity, body_velocity_lin, torque, torque_lin, power, power_lin, ...
    foot_force, foot_force_lin, visualize_lin)
%VISUALIZE_JUMP_DATA - visualize torque, angle, velocities etc. during jump

if (~visualize_lin)
    
    figure('name', 'Jump Analysis')
    subplot(3, 2, 1);
    plot(t, theta,  '-o', 'LineWidth', 3);
    hold on;
    title('Angular Position')
    ylabel('theta [rad]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 3)
    plot(t, theta_dot, '-o', 'LineWidth', 3);
    hold on;
    title('Angular Velocity')
    ylabel('theta\_dot [rad/s]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 5)
    plot(t, body_velocity, '-o', 'LineWidth', 3);
    hold on;
    title('Body Velocity')
    ylabel('v [m/s]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 2)
    plot(t, torque, '-o', 'LineWidth', 3);
    hold on;
    title('Torque')
    ylabel('tau [Nm]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 4)
    plot(t, power, '-o', 'LineWidth', 3);
    hold on;
    title('Power')
    ylabel('P [W]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 6)
    plot(t, foot_force, '-o', 'LineWidth', 3);
    hold on;
    title('Foot Force')
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([0.0 300]);
    grid on;
    
else
    
    figure('name', 'Jump Analysis')
    subplot(3, 2, 1);
    plot(t_lin, theta_lin, 'LineWidth', 3);
    hold on;
    title('Angular Position')
    ylabel('theta [rad]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 3)
    plot(t_lin, theta_dot_lin, 'LineWidth', 3);
    hold on;
    title('Angular Velocity')
    ylabel('theta\_dot [rad/s]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 5)
    plot(t_lin, body_velocity_lin, 'LineWidth', 3);
    hold on;
    title('Body Velocity')
    ylabel('v [m/s]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 2)
    plot(t_lin, torque_lin, 'LineWidth', 3);
    hold on;
    title('Torque')
    ylabel('tau [Nm]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 4)
    plot(t_lin, power_lin, 'LineWidth', 3);
    hold on;
    title('Power')
    ylabel('P [W]');
    xlabel('time [s]');
    grid on;
    
    subplot(3, 2, 6)
    plot(t_lin, foot_force_lin, 'LineWidth', 3);
    hold on;
    title('Foot Force')
    ylabel('F [N]');
    xlabel('time [s]');
    ylim([0.0 300]);
    grid on;
    
    
end

end

