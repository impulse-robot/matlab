%% Analyse jumping height
% Author:   Alex Dietsche
% Date: 19/12/2018


%% Parameters

close all; clear; clc

tau = 0.15; % main motor torque output [Nm]
i = 26.666; % transmission []
m = 0.4; % overall mass of the robot [kg]
g = 9.81; % gravity [m/s^2]
dt = 0.0001; % time step [s]
theta_0 = 3.0; % starting output angle [rad]
u_cell = 4.2; % cell voltage [V] 
n_cell = 4.2; % number of cells []
m_type = 2; % motor type, 1 = F80, 2 = F40

%% Kinematics files

% check if kinamtics files exist
if (~isfile('../data/forward_kinematics.csv') || ~isfile('../data/differential_kinematics.csv'))
    disp('-- Did not find forward_kinematics and / or differential_kinematics file')
    disp('-- Running leg_kinemtics script...')
    run('../leg_kinematics/leg_kinematics.m')
    close all; clear; clc
    disp('-- leg_kinematics script finished.')
end

% Read in forward and differential kinematics
forward_kinematics = csvread('../data/forward_kinematics.csv', 1, 0);
differential_kinematics = csvread('../data/differential_kinematics.csv', 1, 0);

% Account for different coordinate frame in y
forward_kinematics(:, 2) = - forward_kinematics(:, 2);

%% Solve differential equation using eulers method
% initial conditions

y_0 = interp1(forward_kinematics(:, 1), forward_kinematics(:, 2), theta_0);
y_max = forward_kinematics(end, 2) - 0.01;
y_dot_0 = 0;
y_dot_dot_0 = 0;

t = [0]; % time [s]
y = [y_0]; % vertical position [m]
y_dot = [y_dot_0]; % vertical velocity [m/s]
y_dot_dot = [y_dot_dot_0]; % vertical acceleration [m/s^2]
theta = [0]; % output shaft angle [rad]
theta_dot = [0]; % output shaft angular velocity [rad/s]
torque = [0];
jacobian = [];
reaction_force = [0]; % force exerted on the ground [N]
power = [0]; % power of the motor [W]


j = 1;
while true
    
    % solve differential equation for next time step
    y_dot_dot_next = get_y_dot_dot(y(j), y_dot(j));
    y_dot_next = y_dot(j) + dt * y_dot_dot(j);
    y_next = y(j) + dt * y_dot(j);
    
    
    % store results
    t = [t, j * dt];
    y = [y, y_next];
    y_dot = [y_dot, y_dot_next];
    y_dot_dot = [y_dot_dot, y_dot_dot_next];
    theta = [theta, get_theta(y(j))];
    theta_dot = [theta_dot, get_theta_dot(y(j), y_dot(j))];
    
    jacobian = [jacobian, get_jacobian(y(j))];
    
    if (y(j) < y_max)
        torque_next = drehmomentKorrektur(tau, get_theta_dot(y(j), ...
            y_dot(j)) * i, u_cell, n_cell, m_type);
    else
        torque_next = 0;
    end
        
    torque = [torque, torque_next];
    reaction_force = [reaction_force, ground_force(y(j), y_dot(j))];
    power = [power, i * get_theta_dot(y(j), y_dot(j)) * torque(j)];
    
    % end when reaching highest point
    if (y_dot(j) < 0)
        break
    end
    
    j = j + 1;
    
end

% data for visualization
stroke = y - y_0;
rpm = i * 60 * theta_dot / (2 * pi);

% get max jumping height and max velocity
[jumping_height, jumping_height_idx] = max(y);
[y_max_diff, y_max_idx] = min(abs(y - y_max));
[max_velocity, max_velocity_idx] = max(y_dot);
[max_acceleration, max_acceleration_idx] = max(y_dot_dot);
[max_ground_force, max_ground_force_idx] = max(reaction_force);
[max_rpm, max_rpm_idx] = max(rpm);

% plot jumping mechanics
figure('name', 'Jumping Mechanics');
subplot(2, 2, 1);
plot(t, y, 'LineWidth', 3);
hold on;
scatter(t(jumping_height_idx), jumping_height, 50, 'LineWidth', 2);
txt = {'Jump Height:', ['h\_max = ', num2str(jumping_height), 'm']};
text(t(jumping_height_idx) - 0.15, jumping_height - 0.4, txt, 'FontSize', 12);
title('Vertical Position')
ylabel('Height [m]');
xlabel('Time [s]');
grid on;

subplot(2, 2, 2);
plot(t, y_dot, 'LineWidth', 3);
hold on;
scatter(t(max_velocity_idx), max_velocity, 50, 'LineWidth', 2);
txt = {'Max Velocity:', ['v\_max = ', num2str(max_velocity), 'm/s']};
text(t(max_velocity_idx) + 0.2, max_velocity - 1, txt, 'FontSize', 12);
title('Vertical Velocity')
ylabel('Velocity [m/s]');
xlabel('Time [s]');
grid on;

subplot(2, 2, 3);
plot(t, y_dot_dot, 'LineWidth', 3);
hold on;
scatter(t(max_acceleration_idx), max_acceleration, 50, 'LineWidth', 2);
txt = {'Max Acceleration:', ['a\_max = ', num2str(max_acceleration), 'm/s^2']};
text(t(max_acceleration_idx) + 0.2, max_acceleration, txt, 'FontSize', 12);
title('Vertical Acceleration');
ylabel('Acceleration [m/s^2]');
xlabel('Time [s]');
grid on;

subplot(2, 2, 4);
plot(t, reaction_force, 'LineWidth', 3);
hold on;
scatter(t(max_ground_force_idx), max_ground_force, 50, 'LineWidth', 2);
txt = {'Max Ground Force:', ['F\_max = ', num2str(max_ground_force), 'N']};
text(t(max_ground_force_idx) + 0.2, max_ground_force, txt, 'FontSize', 12);
title('Ground Force');
ylabel('Force [N]');
xlabel('Time [s]');
grid on;


% plot motor mechanics
figure('name', 'Motor Mechanics');
subplot(2, 3, 1);
plot(t(1:y_max_idx), torque(1:y_max_idx), 'LineWidth', 3);
title('Motor Torque');
ylabel('Torque [Nm]');
xlabel('Time [s]');
grid on;

subplot(2, 3, 4)
plot(stroke(1:y_max_idx), torque(1:y_max_idx), 'LineWidth', 3);
title('Motor Torque');
ylabel('Torque [Nm]');
xlabel('Stroke [m]');
grid on;

subplot(2, 3, 2)
plot(t(1:y_max_idx), rpm(1:y_max_idx), 'LineWidth', 3);
hold on;
scatter(t(max_rpm_idx), max_rpm, 50, 'LineWidth', 2);
txt = {'Max RPM:', ['rpm = ', num2str(max_rpm)]};
text(t(max_rpm_idx) - 0.02, max_rpm, txt, 'FontSize', 12);
title('Motor RPM');
ylabel('RPM []');
xlabel('Time [s]');
grid on;

subplot(2, 3, 5)
plot(stroke(1:y_max_idx), rpm(1:y_max_idx), 'LineWidth', 3);
title('Motor RPM');
ylabel('RPM []');
xlabel('Stroke [m]');
grid on;

subplot(2, 3, 3)
plot(t(1:y_max_idx), power(1:y_max_idx), 'LineWidth', 3);
title('Motor Power');
ylabel('Power [W]');
xlabel('Time [s]');
grid on;

subplot(2, 3, 6)
plot(stroke(1:y_max_idx), power(1:y_max_idx), 'LineWidth', 3);
title('Motor Power');
ylabel('Power [W]');
xlabel('Stroke [m]');
grid on;


% Mechanical Advantage
figure('name', 'Mechanical Advantage');
subplot(2, 1, 1);
plot(t(1:y_max_idx), jacobian(1:y_max_idx).^(-1), 'LineWidth', 3);
title('Jacobian Inverse');
ylabel('MA [N/Nm]');
xlabel('Time [s]');
grid on;

subplot(2, 1, 2);
plot(stroke(1:y_max_idx), jacobian(1:y_max_idx).^(-1), 'LineWidth', 3);
title('Jacobian Inverse');
ylabel('MA [N/Nm]');
xlabel('Stroke [m]');
grid on;

