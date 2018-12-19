%% Analyse jumping height
% Author:   Alex Dietsche
% Date: 19/12/2018


%% Parameters

close all; clear; clc

tau = 0.2; % main motor torque output [Nm]
i = 25; % transmission []
m = 0.4; % overall mass of the robot [kg]
g = 9.81; % gravity [m/s^2]
dt = 0.001; % time step [s]

%% Kinematics filesfgfg
% check if kinamtics files exist
if (~isfile('forward_kinematics.csv') || ~isfile('differential_kinematics.csv'))
    disp('-- Did not find forward_kinematics and / or differential_kinematics file')
    disp('-- Running leg_kinemtics script...')
    run('leg_kinematics.m')
    close all; clear; clc
    disp('-- leg_kinematics script finished.')
end

% Read in forward and differential kinematics
forward_kinematics = csvread('forward_kinematics.csv', 1, 0);
differential_kinematics = csvread('differential_kinematics.csv', 1, 0);

% Account for different coordinate frame in y
forward_kinematics(:, 2) = - forward_kinematics(:, 2);

%% Solve differential equation using eulers method
% initial conditions
y_0 = forward_kinematics(1, 2);
y_max = forward_kinematics(end, 2) - 0.01;
y_dot_0 = 0;
y_dot_dot_0 = 0;

t = [0];
y = [y_0];
y_dot = [y_dot_0];

j = 1;
while true
    y_dot_next = y_dot(j) + dt * y_dot_dot(y(j), y_dot(j));
    y_next = y(j) + dt * y_dot(j);
    y_dot = [y_dot, y_dot_next];
    y = [y, y_next];
    t = [t, j * dt];
    if (j * dt > 0.8)
        break
    end
    
    j = j + 1;
    
end

% visualize jump
plot(t, y)

